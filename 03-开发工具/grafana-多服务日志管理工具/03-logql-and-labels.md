# 03 — LogQL 与标签参考

本文档面向开发与排障场景，说明如何编写 LogQL 查询、理解标签含义，以及常见问题排查。

完整可复制片段见 [templates/logql-snippets.md](templates/logql-snippets.md)。

## LogQL 基础

LogQL 由两部分组成：

1. **日志流选择器** `{label="value", ...}` — 选择哪些日志流（类似 Prometheus 标签选择）
2. **管道阶段** `| ...` — 在行级别过滤、解析、聚合

示例：

```logql
{product="massage_robot", service_name="robot"} |= "ERROR" | device_id="<device-id>"
```

执行顺序：先按流标签选中日志 → 再按正文/结构化标签过滤。

## 标签字典

### 流标签（必须出现在 `{...}` 中）

| 标签 | Cardinality | 示例 | 用途 |
|------|-------------|------|------|
| `product` | 低 | `massage_robot` | 产品线隔离 |
| `service_name` | 低 | `robot`, `iot`, `ota` | 服务维度 |
| `environment` | 低 | `test`, `prod`（未来） | 环境隔离 |
| `severity` | 低 | `INFO`, `WARN`, `ERROR` | 级别过滤 |

当前线上实例已观测到的 `service_name` 取值：`robot`、`iot`、`ota`。

### 结构化标签（推荐用 `| label=~"$var"` 过滤）

| 标签 | Cardinality | 说明 |
|------|-------------|------|
| `device_id` | 高 | 每台设备唯一，**不要**放入流选择器 |
| `service_version` | 中 | 整机版本，如 `v2.4.6.5` |
| `systemd_unit` | 低 | `massage-robot.service` 等 |
| `process_pid` | 高 | 进程 PID，排障时可用 |
| `journal_cursor` | 极高 | 网关内部去重用 |
| `scope_name` | 低 | 固定 `wlzc-device-log-gateway` |
| `severity_text` | 低 | 与 `severity` 对应的人类可读文本 |
| `severity_number` | 低 | OTel 级别编号（INFO=9） |
| `detected_level` | 低 | 从正文推断的级别 |

## 常用查询模式

### 按服务查

```logql
{product="massage_robot", service_name="robot"}
{product="massage_robot", service_name="iot"}
{product="massage_robot", service_name="ota"}
```

### 按设备查（推荐行过滤）

```logql
{product="massage_robot", service_name="robot"} | device_id="<device-id>"
```

在 Explore 中可先查任意一条日志，从 Labels 面板复制 `device_id`。

### 按关键字查

| 操作符 | 含义 | 示例 |
|--------|------|------|
| `\|=` | 包含子串 | `\|= "trajectory"` |
| `!=` | 不包含 | `!= "Heartbeat"` |
| `\|~` | 正则匹配 | `\|~ "(?i)error\|fail"` |
| `!~` | 正则不匹配 | `!~ "debug"` |

```logql
{service_name="robot"} |= "ERROR"
{service_name="robot"} |~ "(?i)exception|failed|timeout"
```

### 按版本查

```logql
{service_name="robot"} | service_version=~"v2\\.4\\..*"
```

注意：正则中的 `.` 需转义为 `\\.`。

### 聚合统计

```logql
# 5 分钟内按服务、级别计数
sum by (service_name, severity) (
  count_over_time({product="massage_robot"}[5m])
)

# ERROR 速率（可用于告警规则）
sum(rate({product="massage_robot", severity="ERROR"}[5m])) by (service_name, device_id)
```

时间窗口 `[5m]` 需与 Grafana 面板分辨率匹配；Dashboard 使用 `[$__interval]` 自动适配。

## 排障场景手册

### 场景 1：云端看不到刚打的日志

检查清单：

1. **时间范围**：Explore 是否覆盖当前时刻？
2. **environment**：设备是否上报到 `test`？查询是否误过滤了 `prod`？
3. **service_name**：ROS2 日志在 `robot`，不要查 `iot`
4. **延迟**：网关上报通常有秒级延迟，稍等后刷新
5. **本地对比**：设备上 `journalctl -u massage-robot -f` 是否有输出？

### 场景 2：只能看到 INFO，没有 ERROR

可能原因：

- `severity` 流标签由网关解析，若应用只打印 `[ERROR ...]` 文本而未设 stderr 级别，可能仍为 `INFO`
- 改用正文搜索：`{service_name="robot"} |= "ERROR"`

### 场景 3：robot_api 日志在 Grafana 搜不到

`robot_api` 默认写文件 `/var/log/massage_robot/robot_api.log`，不经 journal。只有同时输出到 stdout/stderr 的日志才会进入 Loki。

本地查看：

```bash
tail -f /var/log/massage_robot/robot_api.log
grep ERROR /var/log/massage_robot/robot_api.log
```

若需云端可见，需运维扩展网关采集该文件，或在关键路径增加输出到 stdout。

### 场景 4：跨服务关联排障

一次 OTA 升级可能涉及三个服务：

```logql
{product="massage_robot"} | device_id="<device-id>" | systemd_unit=~"massage-(robot|iot|ota).service"
```

或分 tab 对比 `service_name=robot` / `iot` / `ota`。

### 场景 5：用 trace_id 关联 HTTP 请求

robot_api 在响应头返回 `X-Trace-ID`（格式 `trc-<uuid>`）。若业务代码将 trace_id 写入日志正文，可搜索：

```logql
{service_name="robot"} |= "trc-<uuid>"
```

当前未接入分布式 Trace 后端，无法一键跳转链路视图（见 [05-extension-roadmap.md](05-extension-roadmap.md)）。

## 本地日志与 Loki 对照表

| 你想查的内容 | 本地命令 / 路径 | Grafana 查询 |
|-------------|----------------|--------------|
| ROS2 节点输出 | `journalctl -u massage-robot -f` | `{service_name="robot"}` |
| 服务启动过程 | `cat /var/log/massage_robot/robot_launch.log` | 通常无 |
| FastAPI 详细日志 | `tail -f /var/log/massage_robot/robot_api.log` | 仅 journal 部分 |
| IoT 心跳 | `journalctl -u massage-iot -f` | `{service_name="iot"} \|= heartbeat` |
| OTA 鉴权 | `journalctl -u massage-ota -f` | `{service_name="ota"} \|= auth` |
| 历史日志清理 | `package/clean-logs.sh`（7 天） | Loki 保留策略由运维配置 |

本地日志清理机制见 [package/readme.md](../../package/readme.md#日志自动清理)。

## 性能与查询建议

1. **尽量缩小时间范围**：查最近 15m / 1h，避免一次拉取过多数据
2. **流标签优先**：`{product, service_name, environment, severity}` 比全文扫描快
3. **高基数用行过滤**：`device_id` 用 `| device_id="..."` 而非 `{device_id="..."}`
4. **避免过于宽泛的正则**：`|~ ".*"` 等价于全表扫描

## 下一步

- Dashboard 变量与导入：[04-dashboard-guide.md](04-dashboard-guide.md)
- 告警与架构扩展：[05-extension-roadmap.md](05-extension-roadmap.md)
