# JSON 烧录
```c
# 调用./flash-nvs.sh脚本实现
#1、采用 json 烧录的形式
./tools/flash-nvs.sh --json robot01.json --readback
#{
#  "sn": "robot01",
#  "mac": "80f1b2d2f446",
#  "key": "fcdbec3b591c51a64633e8c4b9812e3e66bcddb8471ee9d0d4777186557e8964"
#}
```

# 指定字段烧录
```c
./tools/flash-nvs.sh \
  --sn robot01 \
  --key fcdbec3b591c51a64633e8c4b9812e3e66bcddb8471ee9d0d4777186557e8964 \
  --mac 80f1b2d2f446 \
  --port /dev/ttyACM0 \
  --readback
```

# 完整 flash-nvs.sh 脚本
```c
#!/usr/bin/env bash

# 烧录 NVS 分区 device 命名空间: sn / key / mac_bind

#

# 用法:

# ./tools/flash-nvs.sh --json robot01.json

# ./tools/flash-nvs.sh --sn robot01 --key <hex> --mac 80f1b2d2f446 --readback

set -euo pipefail

  

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/common.sh

source "${SCRIPT_DIR}/lib/common.sh"

  

NVS_SIZE="0x6000"

NVS_OFFSET="${NVS_OFFSET:-}"

  

usage()

{

cat <<EOF

用法: $0 (--json <file> | --sn <sn> --key <hex> --mac <hex>) [选项]

  

--json <file> 配对 JSON，需含 sn / key / mac 字段

--sn <sn> 设备序列号

--key <hex> MQTT 密码（64 字符 hex）

--mac <hex> 芯片 MAC，12 位十六进制无冒号

  

选项:

--port <dev> 串口（默认 tools/local.env 的 SERIAL_PORT）

--readback 烧录后 read_flash 并打印 NVS 内容

--no-reboot 烧录后不执行 esptool run

-h, --help 显示本帮助

  

示例:

$0 --json robot01.json --readback

$0 --sn robot01 --key aaaa... --mac 80f1b2d2f446 --port /dev/ttyACM0

EOF

}

  

die()

{

echo "错误: $*" >&2

exit 1

}

  

require_esptool()

{

require_idf

python3 -c "import esptool" 2>/dev/null || die "请安装 esptool: pip install esptool"

}

  

nvs_gen_path()

{

local gen="${IDF_PATH}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py"

[[ -f "${gen}" ]] || die "未找到 nvs_partition_gen.py，请 source tools/env.sh"

echo "${gen}"

}

  

# 优先从编译产物读 nvs 偏移，否则默认 0x9000

resolve_nvs_offset()

{

if [[ -n "${NVS_OFFSET}" ]]; then

echo "${NVS_OFFSET}"

return

fi

  

local csv="${PROJECT_ROOT}/build/partition_table/partition-table.csv"

if [[ -f "${csv}" ]]; then

local off

off="$(python3 - <<PY

import csv

from pathlib import Path

with Path("${csv}").open(newline="") as f:

for row in csv.DictReader(f):

if row.get("Name") == "nvs" and row.get("Offset"):

print(row["Offset"])

break

PY

)"

if [[ -n "${off}" ]]; then

echo "${off}"

return

fi

fi

  

echo "0x9000"

}

  

load_from_json()

{

local json_path="$1"

[[ -f "${json_path}" ]] || die "未找到 JSON: ${json_path}"

eval "$(python3 - <<PY

import json

import sys

from pathlib import Path

p = json.loads(Path("${json_path}").resolve().read_text())

for k in ("sn", "mac", "key"):

if k not in p or not p[k]:

sys.exit(f"JSON 缺少字段: {k}")

print(f"SN={p['sn']!r}")

print(f"MAC={p['mac']!r}")

print(f"KEY={p['key']!r}")

PY

)"

}

  

generate_and_flash()

{

local port="$1"

local sn="$2"

local key="$3"

local mac="$4"

local offset="$5"

  

local csv bin gen

csv="$(mktemp --suffix=.csv)"

bin="$(mktemp --suffix=.bin)"

gen="$(nvs_gen_path)"

  

cat > "${csv}" <<EOF

key,type,encoding,value

device,namespace,,

sn,data,string,${sn}

key,data,string,${key}

mac_bind,data,hex2bin,${mac}

EOF

  

python3 "${gen}" generate "${csv}" "${bin}" "${NVS_SIZE}"

[[ -s "${bin}" ]] || die "NVS 镜像为空，generate 可能失败"

  

echo ">>> 烧录 NVS @ ${offset} (sn=${sn}, key=${key:0:8}...)"

python3 -m esptool --port "${port}" write_flash "${offset}" "${bin}"

  

rm -f "${csv}" "${bin}"

}

  

esp_reboot()

{

local port="$1"

echo ">>> 重启 ESP32 (${port})..."

python3 -m esptool --port "${port}" run >/dev/null 2>&1 || true

echo ">>> 等待 8s 让网络与 MQTT 就绪..."

sleep 8

}

  

nvs_readback()

{

local port="$1"

local offset="$2"

local dump tool

  

dump="$(mktemp --suffix=.bin)"

tool="${IDF_PATH}/components/nvs_flash/nvs_partition_tool/nvs_tool.py"

  

echo ">>> 回读 NVS @ ${offset} ..."

python3 -m esptool --port "${port}" read_flash "${offset}" "${NVS_SIZE}" "${dump}"

[[ -f "${tool}" ]] || die "未找到 nvs_tool.py: ${tool}"

python3 "${tool}" -d minimal "${dump}"

rm -f "${dump}"

}

  

main()

{

local json_path="" sn="" key="" mac=""

local port="" do_readback=0 do_reboot=1

  

load_local_env

port="${SERIAL_PORT:-/dev/ttyACM0}"

  

while [[ $# -gt 0 ]]; do

case "$1" in

--json)

json_path="$2"

shift 2

;;

--sn)

sn="$2"

shift 2

;;

--key)

key="$2"

shift 2

;;

--mac)

mac="$2"

shift 2

;;

--port)

port="$2"

shift 2

;;

--readback)

do_readback=1

shift

;;

--no-reboot)

do_reboot=0

shift

;;

-h|--help|help)

usage

exit 0

;;

*)

die "未知参数: $1（用 -h 查看帮助）"

;;

esac

done

  

if [[ -n "${json_path}" ]]; then

load_from_json "${json_path}"

sn="${SN}"

key="${KEY}"

mac="${MAC}"

fi

  

if [[ -z "${sn}" || -z "${key}" || -z "${mac}" ]]; then

usage

die "请指定 --json 或完整的 --sn / --key / --mac"

fi

  

[[ -e "${port}" ]] || echo "警告: 串口 ${port} 不存在" >&2

  

require_esptool

local offset

offset="$(resolve_nvs_offset)"

  

generate_and_flash "${port}" "${sn}" "${key}" "${mac}" "${offset}"

  

if [[ ${do_reboot} -eq 1 ]]; then

esp_reboot "${port}"

fi

  

if [[ ${do_readback} -eq 1 ]]; then

nvs_readback "${port}" "${offset}"

fi

  

echo ">>> NVS 烧录完成"

}

  

main "$@"
```