#!/usr/bin/env python3
"""
render_pseudocode.py
====================
读取 .tex 文件中的 algorithm / algorithmic 环境，
渲染为带 KaTeX 数学公式的 HTML 并在浏览器中打开。

用法:
  python render_pseudocode.py algo_sc_rrt.tex
  python render_pseudocode.py           <- 手动输入路径
"""

import re, sys, os, webbrowser
import html as _HL
from pathlib import Path


# ══════════════════════════════════════════════════════════
# 1.  数学占位符工具
# ══════════════════════════════════════════════════════════
_PH = "\x00MATH{}\x00"

def _protect_math(text):
    store = []
    def _sub(m):
        store.append(m.group(0))
        return _PH.format(len(store) - 1)
    t = re.sub(r'\$\$[\s\S]*?\$\$', _sub, text)
    t = re.sub(r'\$[^\$\n]*?\$',    _sub, t)
    return t, store

def _restore_math(text, store):
    for i, m in enumerate(store):
        text = text.replace(_PH.format(i), m)
    return text


# ══════════════════════════════════════════════════════════
# 2.  LaTeX 文本 → HTML（数学部分原样交给 KaTeX）
# ══════════════════════════════════════════════════════════
def tex2html(text):
    if not text:
        return ''
    t = text.strip()
    # 先整体 HTML 转义（含数学公式内的 < > &）
    # KaTeX auto-render 读取 DOM 文本节点，浏览器会自动反转义 &lt; → <
    t = _HL.escape(t)
    # 再处理 LaTeX 文本命令（\textbf 等不含 HTML 特殊字符，安全替换）
    t = re.sub(r'\\textbf\{([^}]*)\}', r'<strong>\1</strong>', t)
    t = re.sub(r'\\textit\{([^}]*)\}', r'<em>\1</em>',         t)
    t = re.sub(r'\\emph\{([^}]*)\}',   r'<em>\1</em>',         t)
    t = re.sub(r'\\textsc\{([^}]*)\}', r'<span class="sc">\1</span>', t)
    t = re.sub(r'\\texttt\{([^}]*)\}', r'<code>\1</code>',     t)
    # LaTeX 空格命令 → HTML 空格（在 $ 外生效，$ 内 KaTeX 自行处理）
    t = re.sub(r'(?<!\$)~(?!\$)',      '&nbsp;',  t)
    t = re.sub(r'\\quad',              '&emsp;',  t)
    t = re.sub(r'\\,',                 '&thinsp;', t)
    t = re.sub(r'\\;',                 '&ensp;',  t)
    t = re.sub(r'---',                 '&mdash;', t)
    t = re.sub(r'--',                  '&ndash;', t)
    return t


# ══════════════════════════════════════════════════════════
# 3.  花括号 / 注释提取
# ══════════════════════════════════════════════════════════
def _extract_brace(text):
    text = text.lstrip()
    if not text or text[0] != '{':
        end = text.find('\n')
        return (text[:end].strip() if end != -1 else text.strip()), ''
    depth, start = 0, -1
    for i, c in enumerate(text):
        if c == '{':
            if depth == 0:
                start = i + 1
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return text[start:i], text[i+1:]
    return text[1:], ''

def _extract_comment(text):
    idx = text.find('\\Comment{')
    if idx == -1:
        return text, ''
    pre  = text[:idx].strip()
    rest = text[idx + len('\\Comment'):]
    content, _ = _extract_brace(rest)
    return pre, content.strip()


# ══════════════════════════════════════════════════════════
# 4.  解析 algorithmic 环境
# ══════════════════════════════════════════════════════════
_OPEN  = {'If','For','ForAll','While','Repeat','Loop','Procedure','Function'}
_MID   = {'ElsIf','Else'}
_CLOSE = {'EndIf','EndFor','EndWhile','Until','EndLoop','EndProcedure','EndFunction'}
_NONUM = {'Require','Ensure','Input','Output','Statex'}

_KW = {
    'If':           ('if',           'then'),
    'ElsIf':        ('else if',      'then'),
    'Else':         ('else',         ''),
    'EndIf':        ('end if',       ''),
    'For':          ('for',          'do'),
    'ForAll':       ('for all',      'do'),
    'EndFor':       ('end for',      ''),
    'While':        ('while',        'do'),
    'EndWhile':     ('end while',    ''),
    'Repeat':       ('repeat',       ''),
    'Until':        ('until',        ''),
    'Loop':         ('loop',         ''),
    'EndLoop':      ('end loop',     ''),
    'Procedure':    ('Procedure',    ''),
    'EndProcedure': ('end procedure',''),
    'Function':     ('Function',     ''),
    'EndFunction':  ('end function', ''),
    'Return':       ('return',       ''),
    'State':        ('',             ''),
    'Statex':       ('',             ''),
    'Require':      ('Require:',     ''),
    'Ensure':       ('Ensure:',      ''),
    'Input':        ('Input:',       ''),
    'Output':       ('Output:',      ''),
}

_CMD_RE = re.compile(r'\\(' + '|'.join(re.escape(k) for k in _KW) + r')\b')

def parse_algorithmic(body, numbered=True):
    body_safe, store = _protect_math(body)
    body_safe = re.sub(r'(?m)(?<!\\)%.*$', '', body_safe)   # 去 % 注释
    matches = list(_CMD_RE.finditer(body_safe))
    lines, indent, lineno = [], 0, 1

    for i, m in enumerate(matches):
        cmd = m.group(1)
        s   = m.end()
        e   = matches[i+1].start() if i + 1 < len(matches) else len(body_safe)
        raw = _restore_math(body_safe[s:e].strip(), store)
        raw, comment = _extract_comment(raw)
        kw_pre, kw_suf = _KW.get(cmd, ('', ''))
        is_req = cmd in _NONUM

        if cmd in ('If','ElsIf','For','ForAll','While','Until'):
            content, _ = _extract_brace(raw)
        elif cmd in ('Procedure','Function'):
            name, rest2 = _extract_brace(raw)
            params, _   = _extract_brace(rest2)
            content = f'{name}({params})'
        elif cmd in ('Else','EndIf','EndFor','EndWhile',
                     'EndLoop','EndProcedure','EndFunction','Repeat','Loop'):
            content = ''
        else:
            content = raw.lstrip()

        if cmd in _MID or cmd in _CLOSE:
            indent = max(0, indent - 1)

        use_num = numbered and not is_req and cmd != 'Statex'
        lines.append(dict(
            num     = lineno if use_num else None,
            indent  = indent,
            kw_pre  = kw_pre,
            kw_suf  = kw_suf,
            content = content,
            comment = comment,
            is_req  = is_req,
        ))
        if use_num:
            lineno += 1
        if cmd in _OPEN or cmd in _MID:
            indent += 1

    return lines


# ══════════════════════════════════════════════════════════
# 5.  解析整个 .tex 文件
# ══════════════════════════════════════════════════════════
def parse_tex(tex):
    algos = []
    n = 0
    for am in re.finditer(
            r'\\begin\{algorithm\}(?:\[.*?\])?([\s\S]*?)\\end\{algorithm\}', tex):
        n += 1
        body  = am.group(1)
        cap_m = re.search(r'\\caption\{([\s\S]*?)\}', body)
        caption = tex2html(cap_m.group(1)) if cap_m else f'Algorithm {n}'
        alg_m = re.search(
            r'\\begin\{algorithmic\}(?:\[(\d+)\])?([\s\S]*?)\\end\{algorithmic\}', body)
        if alg_m:
            lns = parse_algorithmic(alg_m.group(2), alg_m.group(1) is not None)
        else:
            lns = []
        algos.append(dict(num=n, caption=caption, lines=lns))

    # 裸 algorithmic（无 algorithm 包裹）
    if not algos:
        for alg_m in re.finditer(
                r'\\begin\{algorithmic\}(?:\[(\d+)\])?([\s\S]*?)\\end\{algorithmic\}', tex):
            n += 1
            lns = parse_algorithmic(alg_m.group(2), alg_m.group(1) is not None)
            algos.append(dict(num=n, caption=f'Algorithm {n}', lines=lns))

    return algos


# ══════════════════════════════════════════════════════════
# 6.  生成 HTML
# ══════════════════════════════════════════════════════════
_CSS = """
body{font-family:"Latin Modern","Computer Modern",Georgia,serif;
     background:#efefef;display:flex;flex-direction:column;
     align-items:center;padding:40px 16px;gap:36px}
h1{font-family:serif;color:#222;margin-bottom:0;font-size:1.3em}
.src{color:#888;font-size:12.5px;margin-top:2px}
.algo{background:#fff;border-top:2.5px solid #111;
      border-bottom:2.5px solid #111;width:740px;
      padding:12px 20px 16px;font-size:14.5px;line-height:1.85}
.algo-title{font-weight:bold;font-size:14px;
            border-bottom:1px solid #111;padding-bottom:5px;margin-bottom:7px}
.req-row{display:flex;gap:8px;font-size:13.5px;margin:1px 0}
.req-label{font-weight:bold;min-width:76px;flex-shrink:0}
.sep{border:none;border-top:1px solid #bbb;margin:5px 0}
.line{display:flex;align-items:baseline;min-height:1.85em}
.lnum{min-width:22px;color:#aaa;font-size:11.5px;text-align:right;
      margin-right:10px;user-select:none;flex-shrink:0}
.lbody{flex:1}
.kw{font-weight:bold}
.cm{color:#777;font-style:italic;font-size:12.5px;margin-left:1.2em}
.sc{font-variant:small-caps}
code{font-family:monospace;background:#f4f4f4;padding:0 3px;border-radius:2px}
"""

_KATEX = """
<link rel="stylesheet"
      href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">
<script defer
        src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>
<script defer
        src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js"
        onload="renderMathInElement(document.body,{
          delimiters:[
            {left:'$$',right:'$$',display:true},
            {left:'$', right:'$', display:false}
          ]
        })"></script>
"""

def _algo_to_html(algo):
    rows, prev_req = [], False
    for ln in algo['lines']:
        kw_pre  = ln['kw_pre']
        kw_suf  = ln['kw_suf']
        content = tex2html(ln['content'])
        comment = tex2html(ln['comment'])
        indent  = ln['indent']
        is_req  = ln['is_req']
        if prev_req and not is_req:
            rows.append('<hr class="sep">')
        prev_req = is_req
        if is_req:
            rows.append(
                f'<div class="req-row">'
                f'<span class="req-label">{kw_pre}</span>'
                f'<span>{content}</span></div>'
            )
            continue
        parts = []
        if kw_pre:
            parts.append(f'<span class="kw">{kw_pre}</span>')
        if content:
            parts.append(content)
        if kw_suf:
            parts.append(f'<span class="kw">{kw_suf}</span>')
        if comment:
            parts.append(f'<span class="cm">&#9655; {comment}</span>')
        num_str = str(ln['num']) if ln['num'] is not None else ''
        pad = f'{indent * 1.8:.1f}em'
        rows.append(
            f'<div class="line" style="padding-left:{pad}">'
            f'<span class="lnum">{num_str}</span>'
            f'<span class="lbody">{" ".join(parts)}</span>'
            f'</div>'
        )
    inner = '\n'.join(rows)
    return (
        f'<div class="algo">'
        f'<div class="algo-title">Algorithm&nbsp;{algo["num"]}'
        f'&emsp;{algo["caption"]}</div>'
        f'{inner}</div>'
    )

def build_html(algos, source_path):
    title  = Path(source_path).name
    bodies = '\n'.join(_algo_to_html(a) for a in algos)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>{_HL.escape(title)}</title>
{_KATEX}
<style>{_CSS}</style>
</head>
<body>
<h1>{_HL.escape(title)}</h1>
<p class="src">{_HL.escape(str(source_path))}</p>
{bodies}
</body>
</html>"""


# ══════════════════════════════════════════════════════════
# 7.  入口
# ══════════════════════════════════════════════════════════
def screenshot_html(html_path: Path, png_path: Path):
    """用 Playwright 无头 Chromium 把 HTML 渲染成 PNG"""
    # 优先当前环境，找不到则尝试系统 Python（c:/python310）
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        import subprocess, textwrap
        # 用系统 python 跑一个内联截图脚本
        snippet = textwrap.dedent(f"""
            from playwright.sync_api import sync_playwright
            with sync_playwright() as p:
                b = p.chromium.launch()
                pg = b.new_page(viewport={{"width":900,"height":400}})
                pg.goto(r'{html_path.as_uri()}')
                pg.wait_for_load_state('networkidle', timeout=15000)
                pg.wait_for_timeout(2000)
                h = pg.evaluate('() => document.body.scrollHeight')
                pg.set_viewport_size({{"width":900,"height":h+60}})
                pg.screenshot(path=r'{png_path}', full_page=True)
                b.close()
        """)
        candidates = [
            "c:/python310/python.exe",
            sys.executable,
        ]
        for py in candidates:
            try:
                ret = subprocess.run([py, "-c", snippet], timeout=30,
                                     capture_output=True, text=True)
                if ret.returncode == 0:
                    return True
            except Exception:
                continue
        print("[!] playwright 不可用，跳过截图（pip install playwright && playwright install chromium）")
        return False

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 900, "height": 400})
        page.goto(html_path.as_uri())
        # 等待 KaTeX 脚本加载并渲染完毕
        page.wait_for_load_state("networkidle", timeout=15000)
        page.wait_for_timeout(1500)   # 额外等待 KaTeX auto-render
        # 截取整个页面（全高）
        page.set_viewport_size({"width": 900, "height": 400})
        full_height = page.evaluate("() => document.body.scrollHeight")
        page.set_viewport_size({"width": 900, "height": full_height + 60})
        page.screenshot(path=str(png_path), full_page=True)
        browser.close()
    return True


def main():
    if len(sys.argv) >= 2:
        tex_path = sys.argv[1]
    else:
        tex_path = input("请输入 .tex 文件路径: ").strip().strip('"').strip("'")

    tex_path = Path(tex_path).resolve()
    if not tex_path.exists():
        print(f"[✗] 文件不存在: {tex_path}")
        sys.exit(1)

    tex_content = tex_path.read_text(encoding='utf-8')
    algos = parse_tex(tex_content)

    if not algos:
        print("[✗] 未找到 algorithm / algorithmic 环境，请检查 .tex 文件格式。")
        sys.exit(1)

    print(f"[✓] 解析到 {len(algos)} 个算法环境")
    out_html = tex_path.with_suffix('.html')
    out_html.write_text(build_html(algos, tex_path), encoding='utf-8')
    print(f"[✓] HTML 已生成: {out_html}")

    # 尝试截图生成 PNG
    out_png = tex_path.with_suffix('.png')
    print("[…] 正在用无头浏览器渲染截图，请稍候...")
    ok = screenshot_html(out_html, out_png)
    if ok:
        print(f"[✓] PNG 已生成: {out_png}")
        os.startfile(str(out_png))   # Windows 直接打开图片
    else:
        # 截图失败则降级为浏览器打开
        webbrowser.open(out_html.as_uri())
        print("[✓] 已在浏览器中打开")

if __name__ == '__main__':
    main()

