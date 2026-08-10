# 按摩机器人集中日志体系（Grafana + Loki）

本目录文档说明如何通过公司 Grafana 平台查看多台按摩机器人设备的运行日志。底层为开源方案 [Grafana](https://grafana.com/grafana/) + [Loki](https://grafana.com/oss/loki/)，设备侧由 `wlzc-device-log-gateway` 从 systemd journal 采集并上报云端。

## 5 分钟快速上手

1. 使用公司分配的 Grafana 账号登录：<http://grafana.ware.wlzcrobot.com>
2. 打开 **Explore**，数据源选 **Robot Loki**
3. 输入查询：

```logql
{product="massage_robot", service_name="robot"}
```

4. 选择时间范围（如 Last 5 minutes），点击 **Run query**
5. 或直接进入预置 Dashboard：[机器人设备日志](http://grafana.ware.wlzcrobot.com/d/robot-device-logs)

## 获取访问权限

Grafana 账号由运维/平台团队分配，**请勿在代码或文档中存放明文密码**。只读/查看场景通常分配 Editor 角色即可使用 Explore 与 Dashboard。

## 学习路径（由浅入深）

| 顺序 | 文档 | 适合人群 |
|------|------|----------|
| 1 | [01-quickstart-grafana.md](01-quickstart-grafana.md) | 首次使用 Grafana 的同事 |
| 2 | [02-architecture.md](02-architecture.md) | 需要理解数据从哪来、到哪去 |
| 3 | [03-logql-and-labels.md](03-logql-and-labels.md) | 开发/排障，写自定义查询 |
| 4 | [04-dashboard-guide.md](04-dashboard-guide.md) | 使用或导入 Dashboard 模板 |
| 5 | [05-extension-roadmap.md](05-extension-roadmap.md) | 架构师/运维，规划告警与扩展 |

## 模板与资源

| 文件 | 说明 |
|------|------|
| [templates/dashboard-robot-device-logs.json](templates/dashboard-robot-device-logs.json) | 线上面板 JSON，可导入其他 Grafana |
| [templates/datasource-robot-loki.example.yaml](templates/datasource-robot-loki.example.yaml) | Loki 数据源 provisioning 示例 |
| [templates/logql-snippets.md](templates/logql-snippets.md) | 常用 LogQL 可复制片段 |
| [assets/architecture.mermaid](assets/architecture.mermaid) | 架构图源文件 |

## 与本仓库开发的关系

本仓库（`wlzc_massage_robot_ws`）的 `massage-robot.service` 将 ROS2、robot_api 等进程的 stdout/stderr 写入 **systemd journal**，由设备网关采集后出现在 Loki 的 `service_name=robot` 流中。

开发验证流程建议：

1. **本地先看 journal**：`sudo journalctl -u massage-robot -f`（见 [package/readme.md](../../package/readme.md)）
2. **云端确认上报**：在 Grafana Explore 用 `{service_name="robot"}` 过滤，核对时间戳与内容
3. **注意**：`robot_api` 写入 `/var/log/massage_robot/robot_api.log` 的文件日志**默认不进入 Loki**，除非网关额外采集该路径

## 三个服务名称速查

| `service_name` | 对应进程 | systemd 单元 |
|----------------|----------|--------------|
| `robot` | ROS2 节点 + robot_api（本仓库） | `massage-robot.service` |
| `iot` | massage_iot（云端心跳、MQTT 等） | `massage-iot.service` |
| `ota` | massage_ota（升级与鉴权） | `massage-ota.service` |

## 文档范围说明

本文档基于线上 Grafana API 探测与仓库本地日志机制整理。**不包含** K8s Loki 集群部署、`wlzc-device-log-gateway` 源码与安装细节（需 infra/设备侧仓库权限）。
