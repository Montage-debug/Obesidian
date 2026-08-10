# LogQL 查询片段

可直接复制到 Grafana Explore 或 Dashboard 面板中使用。数据源选择 **Robot Loki**。

---

## 基础选择器

```logql
# 按摩机器人主服务（ROS2 + robot_api）
{product="massage_robot", service_name="robot"}

# IoT 服务（massage_iot，心跳、云端授权等）
{product="massage_robot", service_name="iot"}

# OTA 服务（massage_ota，升级与鉴权）
{product="massage_robot", service_name="ota"}

# 全产品所有服务
{product="massage_robot"}
```

---

## 环境与级别过滤

```logql
# 测试环境
{product="massage_robot", environment="test"}

# 仅 ERROR 级别（流标签）
{product="massage_robot", severity="ERROR"}

# 多级别（正则）
{product="massage_robot", severity=~"ERROR|WARN"}
```

---

## 设备与版本过滤

高基数标签 `device_id`、`service_version` 建议用行过滤器，与 Dashboard 一致：

```logql
# 单台设备（精确）
{product="massage_robot", service_name="robot"} | device_id="<your-device-id>"

# 设备 ID 正则（Dashboard 变量 device_id 默认 .*）
{product="massage_robot", service_name="robot"} | device_id=~"$device_id"

# 指定系统版本
{product="massage_robot", service_name="robot"} | service_version="v2.4.6.5"

# 版本前缀匹配
{product="massage_robot", service_name="robot"} | service_version=~"v2\\.4\\..*"
```

---

## 关键字搜索

```logql
# 包含 ERROR 文本（不限于 severity 标签）
{service_name="robot"} |= "ERROR"

# 排除心跳噪音
{service_name="robot"} != "Heartbeat"

# 正则（大小写不敏感）
{service_name="robot"} |~ "(?i)exception|failed|timeout"

# 多关键字 AND
{service_name="robot"} |= "massage" |= "trajectory"

# IoT 心跳
{service_name="iot"} |= "heartbeat"
```

---

## 按 systemd 单元

```logql
{product="massage_robot"} | systemd_unit="massage-robot.service"
{product="massage_robot"} | systemd_unit="massage-iot.service"
{product="massage_robot"} | systemd_unit="massage-ota.service"
```

---

## 聚合与统计

```logql
# 5 分钟内各服务日志条数（与 Dashboard「日志数量」面板类似）
sum by (service_name, severity) (
  count_over_time({product="massage_robot"}[5m])
)

# 按设备统计
sum by (device_id) (
  count_over_time({product="massage_robot", service_name="robot"}[1h])
)

# ERROR 速率（告警候选）
sum(rate({product="massage_robot", severity="ERROR"}[5m])) by (service_name, device_id)
```

---

## 排障专用

```logql
# 启动相关
{service_name="robot"} |~ "(?i)start|launch|init"

# API 请求失败（robot_api 经 journal 输出）
{service_name="robot"} |= "robot_api" |= "ERROR"

# OTA token 刷新
{service_name="ota"} |= "auth token"

# IoT 心跳
{service_name="iot"} |= "heartbeat"
```

---

## Explore 分享链接

在 Explore 界面配置好查询后，点击 **Share** 生成可分享链接（需有 Grafana 访问权限）。
