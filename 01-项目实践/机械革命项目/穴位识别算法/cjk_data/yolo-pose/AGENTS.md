# AGENTS.md

本文件用于指导在 `/home/cjk/myproject/yolo-pose` 中工作的编码代理。

## 1) 仓库概览

- 语言：Python 脚本（无 package 目录结构）。
- 领域：基于 YOLO 的姿态/穴位推理与可视化。
- 未发现正式项目工具配置（`pyproject.toml`、`requirements.txt`、`setup.py`、`Makefile`、`tox.ini`、`.flake8`、`.github/workflows`、`README`）。
- 运行时资源（`*.pt`、图片文件夹）位于仓库中，由脚本直接使用。

主要脚本：
- `draw_point.py`（背部/腹部穴位主流程）
- `draw_abdomen_point.py`（腹部流程）
- `test.py`（单图推理检查）
- `download_images.py`（数据集辅助脚本）
- `table_json_to_txt.py`（LabelMe -> YOLO 标签）
- `rename.py`（重命名辅助脚本）

## 2) 环境准备

运行环境已经满足，我们安装了anaconda。ultralytics和torch等都已经安装在了anaconda的名为yolo-pose 的虚拟环境下。所以你在测试和运行的时候别在错误的环境中跑。

## 3) 构建 / 检查 / 测试命令

本仓库没有正式的 build/lint/test 框架；请使用脚本级检查。

### 3.1 构建等效检查

```bash
python -m py_compile draw_point.py draw_abdomen_point.py test.py download_images.py table_json_to_txt.py rename.py
```

### 3.2 Lint 等效检查

```bash
python -m compileall .
```

除非明确要求，不要引入仓库级 lint/format 配置。

### 3.3 测试（脚本级）

默认测试命令：

```bash
python test.py
```

当前 `test.py` 行为：
- 加载 `yolo11x-pose.pt`
- 运行于 `leg_data/leg_2.jpg`
- 输出到 `test_result/x/leg_2.jpg`

### 3.4 运行单个测试用例

由于没有测试运行器，“单个测试”即对单张图片运行一次脚本。

推荐方式：
1. 运行 `python test.py`。

可替代的脚本入口：

```bash
python draw_point.py
```

## 4) 代码风格指南（基于仓库现状）

以下约定来自现有代码，应保持一致。

### 4.1 导入

- 导入语句放在文件顶部。
- 当前已存在混合风格（`import x`、`from x import y`）；请与附近文件保持一致。
- 核心脚本常见模式：第三方库（`ultralytics`、`cv2`、`numpy`）+ 标准库。
- 除非必要，避免动态导入。

### 4.2 格式

- 使用 4 空格缩进。
- 在修改文件时保持现有空格风格；避免无关格式化。
- 注释/文档字符串保持简洁；中文领域说明是常见且可接受的。
- 除非明确要求，避免文件级大范围格式整理。

### 4.3 类型与数据形状

- 代码中类型注解较少；不要强行做大规模类型重构。
- 保留基于 NumPy 的计算方式以及 `[x, y, conf]` 元组约定。
- 保持返回值签名不变；很多调用方按元组位置解包。

### 4.4 命名

- 函数/变量优先使用 `snake_case`。
- 数学/配置场景常见全大写常量（例如 `PIXELS_PER_CUN`、`INPUT_DIR`）。
- 领域标识命名要有语义（`acupointCode`、`body_part_code`）。

### 4.5 错误处理与输出

- 遵循现有模式：显式检查 + 早返回进行姿态校验。
- 现有硬错误使用 `ValueError` / `FileNotFoundError`。
- 工具脚本通常使用 `try/except Exception as e` 并配合可读 `print` 输出。
- 默认以 `print(...)` 作为主要反馈方式，不要默认引入日志框架。

## 5) 本仓库代理工作规则

- 优先进行小而精准的目标文件修改。
- 非明确要求，不要重组仓库结构。
- 将模型文件和图片目录视为运行输入，避免删除/移动。
- 非任务需要，不要破坏性清理 `runs/` 或生成结果目录。
- 若新增脚本，保持现有入口风格（`if __name__ == "__main__":`）。
- 强制要求：凡是新增代码或修改代码，必须补充必要备注（至少说明该段代码的目的或关键计算依据）。
- 备注应简洁、贴近代码，就近放置，避免与实现脱节的长篇说明。

## 6) 修改后验证清单

运行：

```bash
python -m py_compile draw_point.py draw_abdomen_point.py test.py download_images.py table_json_to_txt.py rename.py
python test.py
```

如果只改了单个工具脚本，请直接运行该脚本验证。

## 7) Cursor / Copilot 规则

已检查路径：
- `.cursor/rules/`
- `.cursorrules`
- `.github/copilot-instructions.md`

当前仓库快照结果：上述文件/目录均不存在。

## 8) 非目标

- 不要为本仓库凭空引入 `pytest` 命令。
- 不要声称已配置 ruff/flake8/black（除非确实已配置）。
- 执行小修复时，不要引入大范围重构。
