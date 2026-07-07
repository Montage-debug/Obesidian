```python

ESP32(192.168.10.110)---发布/订阅--->MQTT-->Doker-->MQTT<---发布/订阅---PC(192.168.10.121)
```

```c
# Doker控制

# 启动doker
docker compose start

# 临时维护、关机前停止
docker compose stop

# 开机
mosquitto_pub -h 192.168.10.121 -p 1883 \
-u controller -P '0' \
-t 'wlzc/robot01/cmd' -m poweron

# 关机
mosquitto_pub -h 192.168.10.121 -p 1883 \
-u controller -P '0' \
-t 'wlzc/robot01/cmd' -m shutdown

# 若系统自带 mosquitto 占用了 1883，先停掉：
sudo systemctl stop mosquitto
sudo systemctl disable mosquitto

**每天开机后检查 Broker 是否在跑：**
cd ~/Documents/网络唤醒/Doker
docker compose ps
```

```c
# 常用工具
## 0. 首次在新 Linux
```bash
cd wlzc_massage_robot_starter
bash tools/setup-host.sh
cp tools/local.env.example tools/local.env
newgrp dialout

# 1. 每个新终端
cd wlzc_massage_robot_starter
source tools/env.sh
# 2. 编译
./tools/build.sh
# 3. 烧录（推荐，不依赖 Type-C 日志）
./tools/flash.sh
# 4. UDP 日志（推荐验收方式）
./tools/log-udp.sh
# 5. 全片擦除
./tools/erase-flash.sh
```

  


```c
# ESP32
# 终端 1：看设备日志
./tools/log-udp.sh

# 终端 2：看 MQTT 消息
./tools/mqtt-sub.sh all

# 终端 3：发控制
./tools/mqtt-pub.sh poweron
./tools/mqtt-pub.sh shutdown
```

```c
# 执行烧录，用于测试错误的Key
bash -c '
IDF="${IDF_PATH:-$HOME/esp/esp-idf}"
CSV=$(mktemp)
BIN=$(mktemp)
trap "rm -f \"$CSV\" \"$BIN\"" EXIT
cat > "$CSV" <<EOF
key,type,encoding,value
device,namespace,,
sn,data,string,robot01
key,data,string,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
mac_bind,data,hex2bin,80f1b2d2f446
EOF
python3 "$IDF/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py" generate "$CSV" "$BIN" 0x6000
python3 -m esptool --port /dev/ttyACM0 write_flash 0x9000 "$BIN"
'

# 注：仅更改 sn 为当前设备ID 以及 key 中 aaaaaaaa 为自鉴密钥即可
```

```c
# 用以下指令可读取设备中的nvs分区的值
source tools/env.sh

python3 -m esptool --port /dev/ttyACM0 read_flash 0x9000 0x6000 /tmp/nvs_dump.bin

python3 "$IDF_PATH/components/nvs_flash/nvs_partition_tool/nvs_tool.py" -d minimal /tmp/nvs_dump.bin
```

```c
# 检查MQTT是否在运行
mosquitto_sub -h 127.0.0.1 -p 1883 -u controller -P '你的强密码' -t 'wlzc/#' -v
```
