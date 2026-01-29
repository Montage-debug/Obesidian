#!/bin/bash
#
# 快速激活脚本 - 一键激活设备
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo "    WLZC 按摩机器人 - 快速激活"
echo "============================================================"
echo ""

# 运行激活
"${SCRIPT_DIR}/wlzc_license_tool/run.sh" quick-activate

EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "[成功] 授权激活完成"
else
    echo "[失败] 授权激活失败，退出码: $EXIT_CODE"
fi

echo ""
echo "按回车键退出..."
read
