#!/bin/bash
#
# WLZC 授权工具启动脚本
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 如果有 lib 目录，设置 LD_LIBRARY_PATH
if [ -d "${SCRIPT_DIR}/lib" ]; then
    export LD_LIBRARY_PATH="${SCRIPT_DIR}/lib:${LD_LIBRARY_PATH}"
fi

# 运行工具
exec "${SCRIPT_DIR}/license_tool" "$@"
