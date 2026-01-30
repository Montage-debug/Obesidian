# 按摩头控制服务 JSON API 文档

## 概述
本文档描述按摩头管理器的JSON控制服务（`massage_head_control`）和开关服务（`massage_head_switch`）支持的所有命令格式。

### 协议版本说明
- **V1协议**: 旧版按摩头，支持基础功能（温度、LED、电机）
- **V2协议**: 新版机械臂手柄，支持完整功能（RET、微电、负压、发热片、冲击波等）

### 重要说明
V2协议使用统一的 `PresetCommand` 接口，不再使用V1的 `createTemperatureSetCommand()` 接口。所有功能控制都通过档位预设方式实现。

---

## 1. JSON 控制服务 (massage_head_control)

### 服务类型
`robot_interfaces/srv/MassageHeadControl`

### 请求格式
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",  // 按摩头序列号（连字符格式）
  "command_params": "{...}"  // JSON字符串，包含控制参数
}
```

### 支持的控制参数

#### 1.1 RET能量控制（V2协议专用）
**参数名**: `ret`  
**参数范围**: `0` | `10-100`  
**说明**: 控制RET（射频能量传输）功能
- `0`: 停止RET
- `10-100`: 启动RET并预设能量档位（10档最低，100档最高）

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",
  "command_params": "{\"ret\": 50}"
}
```

**命令**: `AA 78 07 32 CC 33 C3 3C`（预设50档）

---

#### 1.2 微电能量控制（V2协议专用）
**参数名**: `micro_electric`  
**参数范围**: `0` | `1-100`  
**说明**: 控制微电功能
- `0`: 停止微电
- `1-100`: 启动微电并预设能量档位

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",
  "command_params": "{\"micro_electric\": 25}"
}
```

**命令**: `AA 78 09 19 CC 33 C3 3C`（预设25档）

---

#### 1.4 负压吸力控制（V2协议专用）
**参数名**: `negative_pressure`  
**参数范围**: `0` | `1-16`  
**说明**: 控制负压手柄的吸力功能
- `0`: 停止负压
- `1-16`: 启动负压并预设吸力档位

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",
  "command_params": "{\"negative_pressure\": 8}"
}
```

**命令**: `AA 78 05 08 CC 33 C3 3C`（预设8档）

---

#### 1.5 发热片温度控制
**参数名**: `temperature`  
**参数范围**: 
- V2协议: `0` | `10-75`（档位）
- V1协议: `0` | `30-60`（°C实际温度）

**说明**: 控制发热片温度
- V2协议: 通过 `PresetCommand::TEMPERATURE` 预设档位（10档最低，75档最高）
- V1协议: 通过温度设置命令控制实际温度值
- `0`: 关闭发热片

**V2协议实现**:
```cpp
// 推荐方式：使用PresetCommand
auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, level);
```

**V1协议实现**:
```cpp
// V1使用温度设置接口
auto cmd = protocol_v1->createTemperatureSetCommand(temperature);
```

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-07",
  "command_params": "{\"temperature\": 35}"
}
```

**命令（V2）**: `AA 78 0D 23 CC 33 C3 3C`（预设35档）  
**命令（V1）**: V1温度控制命令（实际35°C）

---

#### 1.6 电机控制
**参数名**: `motor`  
**参数范围**: `0` | `1` | `2`  
**说明**: 控制旋转手柄的电机
- `0`: 停止电机
- `1`: 电机正转
- `2`: 电机反转

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-05",
  "command_params": "{\"motor\": 1}"
}
```

**命令（V2）**: `AA 78 00 01 CC 33 C3 3C`（正转）

---

#### 1.7 冲击波控制（V2协议专用）
**参数名**: `shock_wave`  
**参数范围**: `>0`（启动） | `0`（停止）  
**说明**: 控制冲击波手柄的冲击波功能

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-08",
  "command_params": "{\"shock_wave\": 100}"
}
```

**命令**: `AA 78 00 5A CC 33 C3 3C`（启停切换）

---

#### 1.8 冲击波频率控制（V2协议专用）
**参数名**: `frequency`  
**参数范围**: `1-21`（Hz）  
**说明**: 设置冲击波频率

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-08",
  "command_params": "{\"frequency\": 10}"
}
```

**命令**: `AA 78 12 0A CC 33 C3 3C`（预设10Hz）

---

#### 1.9 冲击波能量控制（V2协议专用）
**参数名**: `energy`  
**参数范围**: `1-16`  
**说明**: 设置冲击波能量档位

**示例**:
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-08",
  "command_params": "{\"energy\": 5}"
}
```

**命令**: `AA 78 13 05 CC 33 C3 3C`（预设5档）

---

### 1.10 组合控制示例
可在同一请求中包含多个控制参数，系统会按顺序执行：

```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-07",
  "command_params": "{\"ret\": 50, \"temperature\": 40, \"micro_electric\": 30}"
}
```

---

## 2. 开关服务 (massage_head_switch)

### 服务类型
`robot_interfaces/srv/MassageHeadSwitch`

### 请求格式
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",  // 按摩头序列号
  "command": "1"  // 命令代码（字符串格式）
}
```

### 支持的命令代码

| 命令代码 | 功能 | 描述 | 协议支持 |
|---------|------|------|---------|
| `1` | 基础开机 | 执行默认开机序列 | V1/V2 |
| `2` | 停止 | 关闭所有功能 | V1/V2 |
| `3` | RET开机 | 开机并启动RET（默认10档） | V2 |
| `4` | 发热片开机 | 开机并启动发热片（默认35°C） | V1/V2 |
| `5` | 微电开机 | 开机并启动微电（默认25档） | V2 |
| `6` | 全功能开机 | 开机并启动RET+发热片+微电 | V2 |

### 命令示例

#### 2.1 基础开机（命令1）
```bash
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '1'}"
```

#### 2.2 RET开机模式（命令3）
```bash
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '3'}"
```

**执行动作**: 下发 `AA 78 00 10 CC 33 C3 3C`（启动RET）

#### 2.3 发热片开机模式（命令4）
```bash
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-07', command: '4'}"
```

**执行动作**:
- V2协议: 下发 `AA 78 0D 23 CC 33 C3 3C`（预设35°C）
- V1协议: 使用V1温度控制命令

#### 2.4 微电开机模式（命令5）
```bash
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '5'}"
```

**执行动作**: 下发 `AA 78 09 19 CC 33 C3 3C`（预设25档）

#### 2.5 全功能开机模式（命令6）
```bash
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-07', command: '6'}"
```
版本对比与接口设计

### 3.1 V1协议支持的功能
- 电机控制（motor: 0/1/2）
- LED控制（已废弃，V2不支持）
- 温度控制（temperature: 30-60°C实际温度）

### 3.2 V2协议支持的功能
- **全部V1核心功能**（除LED）
- RET能量控制（ret: 10-100档）
- 微电控制（micro_electric: 1-100档）
- 负压控制（negative_pressure: 1-16档）
- 发热片温度（temperature: 10-75档）
- 冲击波控制（shock_wave, frequency: 1-21Hz, energy: 1-16档）

### 3.3 V2协议设计原理

#### ✅ 推荐方式：统一使用PresetCommand接口
V2协议所有功能控制都通过 `createPresetCommand()` 实现：

```cpp
// RET能量控制（10-100档）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::RET_ENERGY, level);

// 发热片温度控制（10-75档）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, level);

// 微电能量控制（1-100档）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::MICRO_ELECTRIC, level);

// 负压吸力控制（1-16档）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::NEGATIVE_PRESSURE, level);

// 冲击波频率控制（1-21Hz）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::FREQUENCY, frequency);

// 冲击波能量控制（1-16档）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::ENERGY, level);
```

#### ⚠️ 已废弃接口：createTemperatureSetCommand()
V1协议使用 `createTemperatureSetCommand()` 控制实际温度值（30-60°C）。  
**V2协议已不再使用此接口**，改为使用 `PresetCommand::TEMPERATURE` 预设档位。

**为什么废弃**：
1. V2协议使用档位控制，不是实际温度值
2. V2的温度范围（10-75档）与V1的温度范围（30-60°C）不兼容
3. 统一使用 `PresetCommand` 接口更清晰、更易维护

**迁移指南**：
```cpp
// ❌ V2协议不推荐（语义混乱）
auto cmd = protocol_->createTemperatureSetCommand(35.0f);

// ✅ V2协议推荐方式（语义清晰）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, 35);
```

### 3.4 协议版本判断
代码中使用 `dynamic_pointer_cast` 判断协议版本：

```cpp
auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
if (protocol_v2) {
    //开始/停止命令的温度控制

### 5.1 开始命令（handleStartCommand）
调用 `start` 命令时，系统会自动启动默认温度：

**V2协议**:
```cpp
// 设置发热片档位30档
auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, 30);
```

**V1协议**:
```cpp
// 设置温度30°C
sendTempCommand(30.0f);
```

### 5.2 停止命令（handleStopCommand）
调用 `stop` 命令时，系统会关闭所有功能：

**V2协议**:
```cpp
// 关闭发热片（档位0）
auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, 0);
```
7. 命令队列机制

所有控制命令会进入队列系统，按FIFO顺序执行：
- **控制命令**: 100ms间隔执行
- **状态命令**: 50ms间隔执行  
- **识别命令**: 50ms间隔执行

建议发送复杂命令组合时预留足够执行时间（每个命令约100-200ms）。

---

## 8. 最佳实践与开发建议

### 8.1 参数命名约定
- **`ret`**: RET能量档位（10-100），相当于射频能量的温度控制
- **`temperature`**: 发热片档位（V2: 10-75档）或温度（V1: 30-60°C）
- 两者概念不同：`ret` 是能量强度，`temperature` 是物理加热

### 8.2 V2协议开发指南

#### ✅ 推荐做法
```cpp
// 1. 判断协议版本
auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);

// 2. V2协议使用PresetCommand
if (protocol_v2) {
    auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, level);
    enqueueControlCommand(cmd, "设置发热片档位");
}

// 3. V1协议保持原有逻辑
else {
    auto cmd = protocol_->createTemperatureSetCommand(temperature);
    enqueueControlCommand(cmd, "设置温度");
}
```

#### ❌ 避免的做法
```cpp
// 不要在V2协议中使用createTemperatureSetCommand
// 这个接口在V2中只能开关RET，不能控制发热片温度
auto cmd = protocol_->createTemperatureSetCommand(35.0f); // 错误！
```

### 8.3 参数验证规则

| 参数 | V1范围 | V2范围 | 单位 |
|------|--------|--------|------|
| `ret` | 不支持 | 0 或 10-100 | 档位 |
| `temperature` | 0 或 30-60 | 0 或 10-75 | V1=°C, V2=档位 |
| `micro_electric` | 不支持 | 0 或 1-100 | 档位 |
| `negative_pressure` | 不支持 | 0 或 1-16 | 档位 |
| `motor` | 0/1/2 | 0/1/2 | 状态码 |

### 8.4 迁移建议
如果你的代码目前使用 `createTemperatureSetCommand()` 控制V2设备，请按以下步骤迁移：

**步骤1**: 添加协议版本判断
```cpp
auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
```

**步骤2**: V2路径使用PresetCommand
```cpp
if (protocol_v2) {
    auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, level);
}
```

**步骤3**: V1路径保持不变
```cpp
else {
    auto cmd = protocol_->createTemperatureSetCommand(temperature);
}
```

---

## 附录A：手柄类型与序列号映射

| 手柄名称 | 识别码(XX) | 完整序列号示例 | 支持功能 | 协议版本 |
|---------|-----------|---------------|---------|---------|
| 指疗手柄 | 04 | 5A-A5-06-83-10-01-00-01-04 | RET+微电 | V2 |
| 旋转手柄 | 05 | 5A-A5-06-83-10-01-00-01-05 | RET+微电+电机 | V2 |
| 负压手柄 | 06 | 5A-A5-06-83-10-01-00-01-06 | RET+微电+负压 | V2 |
| 波纹平手柄 | 07 | 5A-A5-06-83-10-01-00-01-07 | RET+微电+发热 | V2 |
| 冲击波手柄 | 08 | 5A-A5-06-83-10-01-00-01-08 | 冲击波 | V2 |

---

## 附录B：V2协议命令格式速查

### PresetCommand命令格式
固定8字节格式：`AA 78 [P1] [P2] [CMD] CC 33 C3 3C`

| PresetCommand | P1值 | P2=档位/值 | CMD | 功能 |
|--------------|------|-----------|-----|------|
| RET_ENERGY | 0x07 | 10-100 | 0xCC | RET能量预设 |
| MICRO_ELECTRIC | 0x09 | 1-100 | 0xCC | 微电能量预设 |
| NEGATIVE_PRESSURE | 0x05 | 1-16 | 0xCC | 负压吸力预设 |
| TEMPERATURE | 0x0D | 10-75 | 0xCC | 发热片温度预设 |
| FREQUENCY | 0x12 | 1-21 | 0xCC | 冲击波频率预设 |
| ENERGY | 0x13 | 1-16 | 0xCC | 冲击波能量预设 |

### 示例命令
- RET 50档: `AA 78 07 32 CC 33 C3 3C`
- 温度 35档: `AA 78 0D 23 CC 33 C3 3C`
- 微电 25档: `AA 78 09 19 CC 33 C3 3C`
- 负压 8档: `AA 78 05 08 CC 33 C3 3C`

---

## 附录C：修订历史

| 版本 | 日期 | 修改内容 | 作者 |
|------|------|---------|------|
| v2.0 | 2026-01-29 | 废弃V2的createTemperatureSetCommand接口，统一使用PresetCommand | wlzc |
| v1.5 | 2026-01-28 | 添加MassageHeadSwitch多种启动模式 | wlzc |
| v1.0 | 2025-12-XX | 初始版本 | wlzc |

---

**当前版本**: v2.0  
**最后更新**: 2026-01-29  
**维护者**: wlzc  
**技术支持**: massage_head_manager模块"V2温度档位范围错误: 80档 (有效范围: 10-75)"
}
```

**V1协议示例**:
```json
{
  "success": false,
  "result": "V1温度范围错误: 70°C (有效范围: 30-60
### 3.5
### V1协议支持的功能
- 电机控制（motor）
- LED控制（已废弃）
- 温度控制（temperature: 30-60°C）

### V2协议支持的功能
- **全部V1功能**
- RET能量控制（ret）
- 微电控制（micro_electric）
- 负压控制（negative_pressure）
- 发热片温度（temperature: 10-75°C）
- 冲击波控制（shock_wave, frequency, energy）

### 自动降级
当使用V2专用参数控制V1协议设备时，系统会自动忽略不支持的参数并记录DEBUG日志。

---

## 4. 完整示例

### 波纹平手柄（ID=07）全功能启动
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-07",
  "command_params": "{\"ret\": 60, \"temperature\": 42, \"micro_electric\": 40}"
}
```

**执行序列**:
1. `AA 78 07 3C CC 33 C3 3C`（RET能量60档）
2. `AA 78 0D 2A CC 33 C3 3C`（温度42°C）
3. `AA 78 09 28 CC 33 C3 3C`（微电40档）

### 冲击波手柄（ID=08）脉冲治疗
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-08",
  "command_params": "{\"frequency\": 15, \"energy\": 10, \"shock_wave\": 1}"
}
```

**执行序列**:
1. `AA 78 12 0F CC 33 C3 3C`（频率15Hz）
2. `AA 78 13 0A CC 33 C3 3C`（能量10档）
3. `AA 78 00 5A CC 33 C3 3C`（启动冲击波）

---

## 5. 错误处理

### 参数超出范围
系统会返回错误响应并记录错误日志：
```json
{
  "success": false,
  "result": "RET能量超出范围: 150 (应在10-100或0关闭)"
}
```

### 序列号不匹配
请求会被加入挂起队列，等待正确按摩头安装：
```json
{
  "success": false,
  "result": "序列号不匹配，请求已挂起等待正确按摩头安装 (request_id: xxx)"
}
```

### 协议不支持
V2专用功能在V1设备上会被忽略（DEBUG日志）。

---

## 6. 命令队列机制

所有控制命令会进入队列系统，按FIFO顺序执行：
- **控制命令**: 100ms间隔执行
- **状态命令**: 50ms间隔执行  
- **识别命令**: 50ms间隔执行

建议发送复杂命令组合时预留足够执行时间（每个命令约100-200ms）。

---

## 附录：手柄类型与序列号映射

| 手柄名称 | 识别码(XX) | 完整序列号示例 | 支持功能 |
|---------|-----------|---------------|---------|
| 指疗手柄 | 04 | 5A-A5-06-83-10-01-00-01-04 | RET+微电 |
| 旋转手柄 | 05 | 5A-A5-06-83-10-01-00-01-05 | RET+微电+电机 |
| 负压手柄 | 06 | 5A-A5-06-83-10-01-00-01-06 | RET+微电+负压 |
| 波纹平手柄 | 07 | 5A-A5-06-83-10-01-00-01-07 | RET+微电+发热 |
| 冲击波手柄 | 08 | 5A-A5-06-83-10-01-00-01-08 | 冲击波 |

---

**版本**: v2.0  
**更新日期**: 2026-01-29  
**作者**: wlzc
