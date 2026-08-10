# 05 — 扩展路线图

本文档按可行性与依赖关系，列出按摩机器人日志体系的可扩展方向，供产品、开发与运维规划参考。

## 当前能力基线

| 能力 | 状态 |
|------|------|
| 多设备 journal 集中存储 | 已上线 |
| 按 product / service / environment / severity 查询 | 已上线 |
| 设备 ID、版本行级过滤 | 已上线（Dashboard 变量） |
| 预置巡检 Dashboard | 已上线（`robot-device-logs`） |
| 日志告警 | 未配置 |
| 指标（Metrics） | 未接入 |
| 分布式 Trace | 未接入 |
| robot_api 文件日志上云 | 未接入 |

## 短期扩展（仅 Grafana / 查询侧）

**依赖：** 现有 Loki 数据，无需改设备或 K8s。

### 1. 新增专题 Dashboard

| Dashboard 建议 | 查询要点 |
|---------------|----------|
| ERROR 专屏 | `{severity="ERROR"}` 或 `\|= "ERROR"` |
| 按 systemd 单元分栏 | `\| systemd_unit="massage-robot.service"` |
| IoT / OTA 独立视图 | `service_name="iot"` / `"ota"` |
| 单设备详情页 | 固定 `device_id` 变量 + 三服务 Logs 面板 |

实现：复制 [templates/dashboard-robot-device-logs.json](templates/dashboard-robot-device-logs.json)，修改 `panels` 与 `templating`。

### 2. Explore 收藏查询

为团队维护常用查询书签：

- 按摩轨迹相关：`{service_name="robot"} |= "trajectory"`
- 机械臂异常：`{service_name="robot"} |~ "(?i)arm|aubo|elite"`
- 启动失败：`{service_name="robot"} |~ "(?i)launch|fatal"`

### 3. 日志详情与 derived fields

Dashboard Logs 面板已启用 `enableLogDetails`。可进一步配置 **Derived fields**，从正文提取 `device_id`、`trace_id` 等为可点击链接（需 Grafana Admin 配置数据源 `jsonData.derivedFields`）。

## 中期扩展（需运维 / infra 配合）

### 4. Grafana Alerting

基于 Loki 规则发送钉钉、邮件、PagerDuty 等。

示例规则：

```logql
# 5 分钟内某设备 ERROR 日志 > 0
sum by (device_id) (
  count_over_time({product="massage_robot", severity="ERROR"}[5m])
) > 0
```

```logql
# robot 服务日志量骤降（可能进程挂掉）
absent_over_time({product="massage_robot", service_name="robot"}[10m])
```

**注意：** 需评估告警噪音（如周期性 Heartbeat 误判），建议先建 Notification policy 路由到测试群。

### 5. 多环境与 RBAC

当前线上主要可见 `environment=test`。扩展 `prod` 时需：

- 设备网关上报正确 `environment` 标签
- Grafana 按团队配置 Folder 权限（测试环境只读、生产受限）
- Loki 租户或 `product` / `environment` 隔离策略

### 6. 纳入 robot_api 文件日志

**问题：** [`robot_api.log`](../../src/robot_api/robot_api/app/utils/logger.py) 不经 journal，云端不可见。

**方案 A（应用侧，本仓库可改）：** 关键日志同时输出到 stdout（已被 journal 采集）。

**方案 B（设备侧）：** 扩展 `wlzc-device-log-gateway` 或部署 Promtail/Grafana Alloy tail `/var/log/massage_robot/*.log`，打上相同标签后推送 Loki。

**方案 C（混合）：** uvicorn access log 保持文件，业务 ERROR 统一走 stderr。

### 7. 日志保留与成本

与本地 [`clean-logs.sh`](../../package/clean-logs.sh)（7 天）对齐或延长云端保留：

- Loki `retention_period` 配置（如 30d / 90d）
- 冷存储（S3 + compactor）降低长期成本
- 按 `severity` 差异化保留（ERROR 更长、DEBUG 更短）

## 长期扩展（完整可观测性）

### 8. OpenTelemetry Trace + Tempo

本仓库 robot_api 已生成 `X-Trace-ID`（[`request_context.py`](../../src/robot_api/robot_api/app/utils/request_context.py)）。完整链路需要：

1. 应用集成 OTel SDK，导出 Span 到 Tempo
2. Grafana 配置 Trace 数据源
3. Loki 日志与 Trace 通过 `trace_id` 关联（Exemplar 或 derived field）

收益：一次按摩会话从 API → ROS2 → 机械臂的跨进程追踪。

### 9. Prometheus 指标 + 日志联动

| 指标示例 | 来源 |
|----------|------|
| 按摩会话数 / 时长 | robot_api 或 robot_control |
| 机械臂关节力矩 | arm_driver |
| IoT 心跳成功率 | massage_iot |

Grafana 同一 Dashboard 混排 Loki Logs + Prometheus Graph，日志面板点击时间戳跳转指标。

### 10. 多产品线复用

标签模型已预留 `product`。新产品线只需：

- 网关上报 `product=<new_product>`
- Grafana 增加变量或文件夹
- 复用 Dashboard JSON 模板

### 11. AI 辅助排障

将 Loki 查询 API 或 Grafana 集成接入内部工具，支持：

- 自然语言 → LogQL 转换
- 自动汇总某 device_id 最近 ERROR 聚类
- 与机械臂 trace 分析工具互补

## 优先级建议

| 优先级 | 项目 | 负责方 | 预期收益 |
|--------|------|--------|----------|
| P0 | ERROR 专题 Dashboard + Explore 书签 | 开发 | 排障效率立竿见影 |
| P1 | Grafana 告警（ERROR 计数） | 运维 | 主动发现问题 |
| P1 | prod 环境与 RBAC | 运维 | 生产可观测 |
| P2 | robot_api 文件日志上云 | 开发 + 运维 | 补全 API 细节 |
| P2 | 日志保留策略 | 运维 | 成本与合规 |
| P3 | OTel Trace | 开发 + 运维 | 跨服务链路 |
| P3 | Prometheus 指标 | 开发 | 量化 SLO |

## 本仓库可独立推进的改动

无需等待 infra 即可在 `wlzc_massage_robot_ws` 内实施：

1. **关键路径日志打到 stderr**：确保 ERROR 进入 journal 与 Loki
2. **日志中带 trace_id**：`logger.error("... trace_id=%s", get_trace_id())` 便于 Grafana 搜索
3. **文档维护**：随版本更新 `service_version` 说明与排障手册（本目录）

## 相关文档

- 架构：[02-architecture.md](02-architecture.md)
- LogQL：[03-logql-and-labels.md](03-logql-and-labels.md)
- 本地日志：[package/readme.md](../../package/readme.md)
