"""Inspect the ASu workspace or copy its existing editor to an isolated version.

Python 3.10+, standard library only. Does not read browser storage or export PDFs.
"""
from __future__ import annotations

import argparse
import base64
from datetime import date
import hashlib
from html.parser import HTMLParser
import json
import mimetypes
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit
import uuid

ROOT = Path(__file__).resolve().parents[1]


def inside(root: Path, value: str) -> Path:
    path = (root / value).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"Path leaves workspace: {value}")
    return path


def state(root: Path) -> dict:
    return json.loads((root / "workspace-state.json").read_text(encoding="utf-8"))


class Images(HTMLParser):
    def __init__(self):
        super().__init__()
        self.sources = []

    def handle_starttag(self, tag, attrs):
        if tag == "img":
            src = dict(attrs).get("src")
            if src:
                self.sources.append(src)


def image_sources(html: str) -> list[str]:
    parser = Images()
    parser.feed(html)
    return list(dict.fromkeys(parser.sources))


def local_image(base: Path, src: str, root: Path) -> Path:
    parsed = urlsplit(src)
    if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment:
        raise ValueError(f"Unsupported local image URL: {src}")
    path = (base / unquote(parsed.path)).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError("Image leaves workspace")
    if not path.is_file():
        raise ValueError(f"Missing image: {src}")
    return path


def check(root: Path) -> int:
    config = state(root)
    failures = []
    for name in ("asu", "asu-resume", "resume", "offer", "contributor"):
        path = root / "skills" / name / "SKILL.md"
        if not path.is_file():
            failures.append(f"Missing skill: {name}")
            continue
        text = path.read_text(encoding="utf-8-sig")
        front = re.match(r"\A---\n(.*?)\n---\n", text, re.S)
        if not front or not re.search(rf"^name: {re.escape(name)}$", front[1], re.M):
            failures.append(f"Invalid skill name/frontmatter: {name}")
        if not front or not re.search(r"^description: \S", front[1], re.M):
            failures.append(f"Missing description: {name}")
        # Validate actual Markdown links; code-form resource examples are optional.
        for link in re.findall(r"(?<!!)\[[^\]]+\]\(([^)]+)\)", text):
            if not urlsplit(link).scheme and not link.startswith("#"):
                if not (path.parent / unquote(link.split("#")[0])).is_file():
                    failures.append(f"Broken skill link: {name}: {link}")
    baseline = inside(root, config["baseline_html"])
    if not baseline.is_file():
        failures.append("Baseline HTML missing")
    else:
        html = baseline.read_text(encoding="utf-8")
        for src in image_sources(html):
            if not src.startswith("data:"):
                try:
                    local_image(baseline.parent, src, root)
                except ValueError as error:
                    failures.append(str(error))
    for label, relative in (
        ("Original ASu master", "assets/asu-resume-template.html"),
        ("General template library", "assets/templates-html"),
        ("Offer tracker master", "assets/application-tracker.html"),
    ):
        print(f"RESOURCE {'present' if (root / relative).exists() else 'missing (fallback documented)'}: {label}")
    for error in failures:
        print(f"FAIL: {error}")
    print(f"CHECK: {'FAIL' if failures else 'PASS'}; structural checks only, not print/PDF QA")
    return int(bool(failures))


def prepare_html(source: Path, root: Path, token: str) -> str:
    html = source.read_text(encoding="utf-8")
    # Apply user-confirmed contact corrections even when copying a historical source.
    for old, corrected in state(root).get("contact_corrections", {}).items():
        html = html.replace(old, corrected)
    pattern = r"const\s+key\s*=\s*(['\"])[^'\"\r\n]+\1\s*;"
    if len(re.findall(pattern, html)) != 1:
        raise ValueError("Unsupported editor: expected exactly one const key storage declaration")
    mode_pattern = r"(['\"])resume-page-mode(?:-[a-f0-9]+)?\1"
    if len(re.findall(mode_pattern, html)) != 2:
        raise ValueError("Unsupported editor: expected two page-mode storage references")
    calls = re.findall(r"localStorage\.(?:getItem|setItem)\(\s*([^,)]+)", html)
    if len(calls) != 4 or any(
        arg.strip() != "key" and not re.fullmatch(mode_pattern, arg.strip()) for arg in calls
    ):
        raise ValueError("Unsupported editor: storage calls require manual review")
    html = re.sub(pattern, f"const key='resume-version-{token}';", html, count=1)
    html = re.sub(mode_pattern, f"'resume-page-mode-{token}'", html)
    for src in image_sources(html):
        if src.startswith("data:"):
            continue
        path = local_image(source.parent, src, root)
        mime = mimetypes.guess_type(path.name)[0]
        if not mime or not mime.startswith("image/"):
            raise ValueError(f"Unsupported image type: {path.name}")
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        # Replace both DOM source and current editor's JS photoSrc/error fallback.
        data = f"data:{mime};base64,{encoded}"
        for spelling in sorted({src, "./" + src.removeprefix("./")}, key=len, reverse=True):
            html = html.replace(spelling, data)
    return html


def new_version(root: Path, direction: str, day: str, source_arg: str | None) -> Path:
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", day):
        raise ValueError("Date must be YYYY-MM-DD")
    date.fromisoformat(day)
    if not re.fullmatch(r"[\w\-\u4e00-\u9fff]{1,60}", direction):
        raise ValueError("Direction must use 1-60 letters, digits, underscores or hyphens")
    config = state(root)
    source = inside(root, source_arg or config["baseline_html"])
    if source.suffix.lower() != ".html" or not source.is_file():
        raise ValueError("Source must be an existing workspace HTML")
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    html = prepare_html(source, root, uuid.uuid4().hex)
    versions = inside(root, config["versions_directory"])
    versions.mkdir(parents=True, exist_ok=True)
    for number in range(1, 10000):
        destination = inside(root, str(versions / f"{day}-{direction}-v{number:02d}"))
        try:
            destination.mkdir()
            break
        except FileExistsError:
            continue
    else:
        raise ValueError("No available version directory")
    (destination / "resume.html").write_text(html, encoding="utf-8")
    note = (
        f"# {day} · {direction} · v{number:02d}\n\n"
        f"- 编辑来源：`{source.relative_to(root).as_posix()}`\n"
        f"- 来源 SHA256：`{source_hash}`\n"
        "- 当前状态：仅复制，尚未针对岗位改写或完成事实核对。\n"
        "- 技术处理：照片已嵌入；内容及页面模式存储键已独立。\n"
        "- JD 来源：待提供。\n"
        "- 本次修改：待记录。\n"
        "- 证据及待确认项：参见 workspace-state.json，按实际修改补充。\n"
        "- 打印/PDF 验证：未完成；本命令不生成 PDF、不修复原版布局。\n\n"
        "浏览器里尚未写回磁盘的编辑不包含在此副本中。\n"
    )
    (destination / "版本说明.md").write_text(note, encoding="utf-8")
    return destination


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="Workspace root (also supports isolated checks)")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="Read-only check of entrypoints and baseline images")
    new = sub.add_parser("new", help="Copy the existing editor with isolated storage and embedded images")
    new.add_argument("--direction", required=True)
    new.add_argument("--date", default=date.today().isoformat())
    new.add_argument("--source", help="Workspace-relative source HTML; defaults to the registered baseline")
    args = parser.parse_args()
    try:
        root = args.root.resolve()
        if args.command == "check":
            return check(root)
        result = new_version(root, args.direction, args.date, args.source)
        print(result)
        return 0
    except (OSError, ValueError, KeyError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
