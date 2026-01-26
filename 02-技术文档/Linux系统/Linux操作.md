
# 网关IP设置

![[Pasted image 20260122162638.png]]

```shell
# 常用监听指令

# 过滤需要的服务
grep 服务名

# 过滤掉不需要的服务，|过滤多个
# 可以加入-v -E -i
grep -v （服务名1|服务名2）
journalctl -u massage-robot.service -f | grep -v "(API 请求完成|massage_process_info_get|API 请求开 始|robot_api.app.utils.ros_accessor)"

# 过滤字段
journalctl -u massage-robot.service -f | grep -v -E -i "(api|API|music|speech)"

```



```c
大多数中文输入法默认支持「简繁切换」快捷键，尝试以下操作：
搜狗输入法（Linux 版）：按下 Ctrl + Shift + F 组合键，切换简繁体状态（切换时会有提示框显示当前状态）。
ibus 框架输入法（如 ibus-pinyin、ibus-libpinyin）：通常也是 Ctrl + Shift + F，部分版本可能需要在设置中确认快捷键。
fcitx 框架输入法（如 fcitx-sunpinyin、fcitx-googlepinyin）：默认快捷键多为 Ctrl + Shift + F 或 Ctrl + F，可在设置中查看。
```

# 调整键盘backspace速度

```c
**gsettings set org.gnome.desktop.peripherals.keyboard delay 1000**
```



```
# 截图gnome插件实现
```markdown
# GNOME 桌面环境截图指令指南

## 主要工具
`gnome-screenshot` - GNOME 桌面环境自带的截图工具

## 常用指令及功能

### 1. 截取全屏并保存到默认路径
```bash
gnome-screenshot
```
- **功能**：截取整个屏幕，默认保存到 `~/图片/` 目录
- **文件名格式**：`Screenshot-YYYYMMDD-HHMMSS.png`
### 2. 截取全屏并复制到剪贴板
```bash
gnome-screenshot -c
```
- **选项说明**：`-c` 表示「复制到剪贴板」
- **功能**：截图后可直接粘贴到文档、聊天窗口等

### 3. 截取指定窗口
```bash
gnome-screenshot -w
```
- **选项说明**：`-w` 表示「窗口模式」
- **操作**：运行后鼠标变成十字准星，点击选择窗口
- **组合使用**：`gnome-screenshot -w -c`（截取窗口并复制到剪贴板）

### 4. 截取自定义区域
```bash
gnome-screenshot -a
```
- **选项说明**：`-a` 表示「区域模式」
- **操作**：拖拽鼠标选择需要截取的区域
- **常用组合**：`gnome-screenshot -a -c`（截取区域并复制到剪贴板）

### 5. 延迟截图
```bash
gnome-screenshot -d 5
```
- **选项说明**：`-d N` 表示延迟 `N` 秒后截图
- **适用场景**：需要先操作界面再截图的场景（如菜单、弹窗）
- **示例**：`gnome-screenshot -a -c -d 3`（延迟3秒截取区域并复制）

### 6. 指定保存路径和文件名
```bash
gnome-screenshot -f ~/Downloads/my_screenshot.png
```
- **选项说明**：`-f 路径/文件名` 自定义保存位置和名称
- **注意**：需确保路径存在
- **示例**：`gnome-screenshot -a -f ~/文档/part.png`

## 快捷键操作（推荐）
以下快捷键比手动输入指令更高效：

| 快捷键 | 功能 |
|--------|------|
| `PrtSc` | 截取全屏并保存到 `~/图片/` |
| `Shift + PrtSc` | 截取自定义区域并保存 |
| `Alt + PrtSc` | 截取当前窗口并保存 |
| `Ctrl + PrtSc` | 截取全屏并复制到剪贴板 |
| `Ctrl + Shift + PrtSc` | 截取自定义区域并复制到剪贴板 |
| `Ctrl + Alt + PrtSc` | 截取当前窗口并复制到剪贴板 |

## 工具补充说明
- `gnome-screenshot` 已能满足大部分基础截图需求
- 如需更复杂功能（如标注、编辑等），可安装 `flameshot` 等第三方工具
- 所有指令均原生适配 GNOME 环境




```shell
#ln -f 源文件 软链接文件存放路径
ln -f 源文件 链接名称
# 强制覆盖已存在的符号链接
ln -sf new_source.txt existing_symlink  
```


```shell
# 实时监控文件内容
tail -f /var/log/system.log
# 显示最后20行
tail -20 filename.txt
```