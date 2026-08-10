# 04 — Dashboard 使用与模板导入

本文档说明线上「机器人设备日志」Dashboard 的结构、变量用法，以及如何将模板导入到其他 Grafana 实例。

## 线上面板概览

| 属性 | 值 |
|------|-----|
| 标题 | 机器人设备日志 |
| UID | `robot-device-logs` |
| 文件夹 | Robot Observability |
| 刷新间隔 | 30s |
| 默认时间范围 | Last 6 hours |
| 数据源 | Robot Loki（`robot-loki`） |

访问：<http://grafana.ware.wlzcrobot.com/d/robot-device-logs>

## 面板结构

### 面板 1：日志数量（Timeseries）

**查询：**

```logql
sum by (service_name, severity) (
  count_over_time(
    {environment=~"$environment", service_name=~"$service", severity=~"$severity"}
    | device_id=~"$device_id"
    | service_version=~"$system_version"
    [$__interval]
  )
)
```

**用途：** 观察各服务、各级别日志量随时间变化，快速发现异常尖峰。

### 面板 2：设备日志（Logs）

**查询：**

```logql
{environment=~"$environment", service_name=~"$service", severity=~"$severity"}
| device_id=~"$device_id"
| service_version=~"$system_version"
```

**选项：**

- 按时间倒序（Descending）
- 启用日志详情（`enableLogDetails: true`）
- 显示标签（`showLabels: true`）

**用途：** 查看原始日志正文，点击单行展开完整标签。

## 模板变量

| 变量名 | 显示名 | 类型 | 来源 / 默认值 | 说明 |
|--------|--------|------|---------------|------|
| `environment` | 环境 | Query | `label_values(environment)` | 支持 All |
| `service` | 服务 | Query | `label_values(service_name)` | robot / iot / ota |
| `severity` | 级别 | Query | `label_values(severity)` | INFO / ERROR 等 |
| `device_id` | 设备 ID（正则） | Textbox | `.*` | 填完整 ID 或正则片段 |
| `system_version` | 系统版本（正则） | Textbox | `.*` | 如 `v2.4.*` |

### 变量使用技巧

**查单台设备：** 将「设备 ID」改为完整 device_id（无需正则特殊字符时直接粘贴）。

**查某版本所有设备：** 环境选 `test`，系统版本填 `v2\.4\.6\.5` 或 `v2\.4\..*`。

**排除心跳噪音：** 在 Logs 面板可临时 Edit → 查询追加 `!= "Heartbeat"`（需 Editor 权限；线上面板 `editable: false` 时请在 Explore 中操作）。

## 导入模板到其他 Grafana

模板文件：[templates/dashboard-robot-device-logs.json](templates/dashboard-robot-device-logs.json)

### 前提

1. 目标 Grafana 已配置 Loki 数据源
2. 数据源 **uid 必须为 `robot-loki`**，或导入后手动改面板数据源

数据源配置参考：[templates/datasource-robot-loki.example.yaml](templates/datasource-robot-loki.example.yaml)

### 导入步骤

1. Grafana → **Dashboards** → **New** → **Import**
2. 点击 **Upload JSON file**，选择 `dashboard-robot-device-logs.json`
3. 选择目标文件夹（可新建 `Robot Observability`）
4. 确认数据源映射为 Loki → **Import**

### 导入后检查

- 打开变量下拉框，确认 `environment`、`service`、`severity` 有数据
- 若变量为空，检查 Loki 中是否存在对应标签
- 若面板报 datasource not found，将面板数据源 uid 改为你的 Loki uid

### JSON 关键字段说明

```json
{
  "uid": "robot-device-logs",
  "title": "机器人设备日志",
  "tags": ["robot", "device-logs"],
  "templating": {
    "list": [ /* 五个变量定义 */ ]
  },
  "panels": [ /* 时序图 + 日志流 */ ]
}
```

导出时已移除服务端 `id` 字段，避免与目标实例冲突。

## 自建 Grafana 最小清单

若需在测试环境复现查询能力：

1. 部署 Loki（单机或 K8s Helm）
2. 部署 Grafana，provisioning 数据源（见 example yaml）
3. 导入 Dashboard JSON
4. 确保设备网关或 Promtail 向 Loki 推送带相同标签结构的日志

标签约定应与线上一致：

```
product=massage_robot
service_name=robot|iot|ota
environment=test
severity=INFO|ERROR|...
```

结构化标签：`device_id`, `service_version`, `systemd_unit` 等。

## 权限说明

线上查看类账号通常为 **Editor**：可查看 Dashboard、使用 Explore、创建个人 Dashboard，但**不能**修改 provisioning 托管的面板（`editable: false`）。

如需修改线上面板定义，联系 Grafana Admin 更新 provisioning 源或放开编辑权限。

## 下一步

- 规划告警与多环境：[05-extension-roadmap.md](05-extension-roadmap.md)
- LogQL 深入：[03-logql-and-labels.md](03-logql-and-labels.md)
