# 01 — Grafana 快速入门

本文档帮助你在 10 分钟内完成：登录 Grafana → 查询一台设备的 robot 日志 → 使用预置 Dashboard。

## 1. 登录

1. 浏览器打开：<http://grafana.ware.wlzcrobot.com>
2. 输入公司分配的账号密码
3. 登录后默认组织为 **Main Org.**

若页面提示静态资源加载失败，多为反向代理配置问题，联系运维；API 与 Explore 通常仍可用。

## 2. 使用 Explore 查日志

Explore 是 Grafana 的即席查询界面，适合排障时快速搜日志。

### 步骤

1. 左侧菜单点击 **Explore**（指南针图标）
2. 顶部数据源选择 **Robot Loki**（uid: `robot-loki`）
3. 切换到 **Code** 编辑模式，输入 LogQL：

```logql
{product="massage_robot", service_name="robot"}
```

4. 右上角时间范围选 **Last 5 minutes** 或 **Last 1 hour**
5. 点击 **Run query**

### 常用过滤

在查询后追加行过滤器（管道 `|`）：

```logql
# 指定设备（将 device_id 替换为实际值，可在日志详情标签中查看）
{product="massage_robot", service_name="robot"} | device_id="<device-id>"

# 搜索关键字
{product="massage_robot", service_name="robot"} |= "ERROR"
```

更多查询见 [templates/logql-snippets.md](templates/logql-snippets.md) 与 [03-logql-and-labels.md](03-logql-and-labels.md)。

### 读懂一条日志

日志行左侧可展开 **Labels**，常见字段：

| 标签 | 含义 |
|------|------|
| `product` | 产品线，固定 `massage_robot` |
| `service_name` | 服务：`robot` / `iot` / `ota` |
| `environment` | 环境，如 `test` |
| `device_id` | 设备唯一 ID |
| `service_version` | 整机软件版本，如 `v2.4.6.5` |
| `severity` | 级别：`INFO`、`ERROR` 等 |
| `systemd_unit` | 来源 systemd 单元 |

日志正文示例：

```
[robot_control_node-11] [INFO 2026-08-10 17:57:08.598] [robot_control_node]: [SignalDetector] Heartbeat
```

Rust 服务（iot/ota）可能带 ANSI 颜色码，Grafana 会原样显示，不影响搜索。

## 3. 使用 Dashboard「机器人设备日志」

预置 Dashboard 适合日常巡检多台设备。

### 打开方式

- 直接访问：<http://grafana.ware.wlzcrobot.com/d/robot-device-logs>
- 或：左侧 **Dashboards** → 文件夹 **Robot Observability** → **机器人设备日志**

### 顶部变量（过滤器）

| 变量 | 类型 | 说明 |
|------|------|------|
| 环境 | 下拉 | 来自 Loki `environment` 标签，可选 All |
| 服务 | 下拉 | `service_name`：`robot` / `iot` / `ota` |
| 级别 | 下拉 | `severity`：`INFO` / `ERROR` 等 |
| 设备 ID（正则） | 文本框 | 默认 `.*` 表示全部；填具体 ID 可精确过滤 |
| 系统版本（正则） | 文本框 | 默认 `.*`；可填 `v2.4.*` 等 |

修改变量后 Dashboard 自动刷新（间隔 30 秒）。

### 两个面板

1. **日志数量**（时序图）  
   按 `service_name`、`severity` 统计日志条数，用于观察流量突增或 ERROR 尖峰。

2. **设备日志**（日志流）  
   显示原始日志，支持展开详情、按时间倒序。点击一行可查看完整标签。

## 4. 分享查询

在 Explore 中配置好查询后：

1. 点击右上角 **Share**
2. 选择 **Copy link** 或 **Snapshot**（快照不依赖实时数据）

可将链接发给同事（对方需有 Grafana 访问权限）。

## 5. 与本地 journal 对照

设备现场排障时，可 SSH 到机器人执行：

```bash
sudo journalctl -u massage-robot -f
```

云端 Grafana 看到的是同一 journal 经网关上报后的副本，通常有数秒到数十秒延迟。本地命令详见 [package/readme.md](../../package/readme.md)。

## 下一步

- 理解架构与数据流：[02-architecture.md](02-architecture.md)
- 深入学习 LogQL：[03-logql-and-labels.md](03-logql-and-labels.md)
