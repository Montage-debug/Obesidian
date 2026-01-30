# JSON控制服务完整实现文档

## 文档概述
本文档详细说明 `massage_head_control` JSON控制服务的完整实现机制，包括数据库查询、序列号验证、协议分发和命令执行的全流程。

---

## 1. 服务架构总览

### 1.1 服务定义
**服务名称**: `/massage_head_control`  
**服务类型**: `robot_interfaces::srv::MassageHeadControl`

**请求结构**:
```cpp
string serial_number    # 按摩头序列号（连字符格式："5A-A5-06-83-10-01-00-01-06"）
string command_params   # JSON字符串，包含控制参数
```

**响应结构**:
```cpp
bool success           # 执行是否成功
string result          # 执行结果描述
```

### 1.2 实现入口
位于 [massage_head_manager.cpp](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L951-L987):
```cpp
void MassageHeadManageNode::handle_json_service(
    const std::shared_ptr<MassageHeadControlsrv::Request> request,
    const std::shared_ptr<MassageHeadControlsrv::Response> response)
```

---

## 2. 核心实现流程

### 2.1 完整调用链
```
handle_json_service()
    ↓
handle_json_control_service()
    ↓ (序列号格式转换)
utils::hexHyphensToSpaces("5A-A5-06...") → "5A A5 06..."
    ↓ (序列号验证)
validateSerialNumberMapping(serial_number)
    ↓ (比较当前在线设备序列号)
current_serial_number_ == serial_number ?
    ↓ YES → 继续执行
    ↓ NO  → 返回错误 + 发布不匹配事件
    ↓
解析JSON参数 (nlohmann::json)
    ↓
按参数类型分发控制命令
    ↓
├─ ret → RET能量控制
├─ micro_electric → 微电能量控制
├─ negative_pressure → 负压吸力控制
├─ temperature → 温度控制
├─ motor → 电机控制
├─ shock_wave → 冲击波脉冲控制
├─ frequency → 冲击波频率控制
└─ energy → 冲击波能量控制
    ↓
protocol_v2->create***Command()
    ↓
enqueueControlCommand() → 命令队列
    ↓
sendSerialData() → 串口发送
```

---

## 3. 序列号验证机制

### 3.1 格式转换
JSON请求使用连字符格式，内部使用空格格式：

```cpp
// 输入格式："5A-A5-06-83-10-01-00-01-06"
std::string serial_number = utils::hexHyphensToSpaces(request->serial_number);
// 输出格式："5A A5 06 83 10 01 00 01 06"
```

**实现位置**: [massage_head_manager.cpp#L389](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L389)

### 3.2 在线设备验证
```cpp
bool MassageHeadManageNode::validateSerialNumberMapping(const std::string& requested_serial_number)
{
    // 1. 获取当前连接的按摩头序列号
    std::string current_serial = current_serial_number_;

    if (current_serial.empty()) {
        RCLCPP_WARN(this->get_logger(), "当前没有按摩头连接");
        return false;
    }

    // 2. 比较序列号
    if (requested_serial_number != current_serial) {
        RCLCPP_WARN(this->get_logger(), "序列号不匹配: 请求=%s, 实际=%s",
                   requested_serial_number.c_str(), current_serial.c_str());

        // 发布不匹配事件
        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
        event_msg->header.stamp = this->now();
        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial);
        event_msg->is_attached = 1;
        event_msg->is_matched = false;
        event_pub_->publish(*event_msg);

        return false;
    }

    return true;
}
```

**实现位置**: [massage_head_manager.cpp#L310-L343](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L310-L343)

**关键字段**: `current_serial_number_`
- 更新时机：串口接收到手柄识别帧（`5A A5 06 83 10 01 00 01 XX`）时更新
- 更新位置：[massage_head_manager.cpp#L1127-L1191](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L1127-L1191)

---

## 4. 数据库查询机制

### 4.1 数据库结构
**表名**: `massage_head`

**关键字段**:
```sql
CREATE TABLE massage_head (
    id TEXT PRIMARY KEY,                    -- UUID
    serial_number TEXT UNIQUE NOT NULL,     -- 序列号（连字符格式）
    name TEXT NOT NULL,                     -- 按摩头名称（如"负压手柄"）
    config_data TEXT,                       -- JSON配置（TCP坐标、负载等）
    switch_command TEXT,                    -- 开关控制命令JSON
    image_url TEXT,
    icon_url TEXT,
    created_at DATETIME,
    updated_at DATETIME
);
```

### 4.2 数据库初始化与缓存
**初始化流程**:
```cpp
// 1. 构造时创建数据库管理器
database_manager_ = std::make_unique<DatabaseManager>(this->get_logger());

// 2. 初始化数据库连接
database_manager_->initialize(db_path);

// 3. 加载所有数据到内存缓存
loadDatabaseCache();
```

**缓存结构**:
```cpp
struct DatabaseCache {
    bool is_loaded = false;
    std::map<std::string, MassageHeadInfo> serial_to_info;  // 序列号→按摩头信息
    std::map<std::string, TcpPosition> serial_to_tcp;       // 序列号→TCP坐标
};
```

**实现位置**: [massage_head_manager.cpp#L346-L387](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L346-L387)

### 4.3 数据库查询方法
**核心查询接口** (DatabaseManager):
```cpp
// 通过序列号查询按摩头信息
std::vector<MassageHead> getMassageHeadBySerialNumber(const std::string& serial_number);

// 通过序列号查询TCP坐标
std::map<std::string, TcpPosition> getMassageHeadTcpBySerialNumber(const std::string& serial_number);

// 从缓存获取（优先）
MassageHead getMassageHeadFromCache(const std::string& serial_number);
std::map<std::string, TcpPosition> getTcpFromCache(const std::string& serial_number);
```

**查询优先级**: 内存缓存 > 数据库查询

---

## 5. JSON参数解析

### 5.1 支持的参数格式
**格式1**: 扁平式（推荐）
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",
  "command_params": "{\"ret\": 50, \"micro_electric\": 25, \"temperature\": 40}"
}
```

**格式2**: 嵌套式（兼容）
```json
{
  "serial_number": "5A-A5-06-83-10-01-00-01-06",
  "command_params": "{\"properties\": [{\"ret\": 50}, {\"micro_electric\": 25}]}"
}
```

### 5.2 解析实现
```cpp
nlohmann::json params;
nlohmann::json json_data = nlohmann::json::parse(request->command_params);

// 自动适配格式
if (json_data.contains("properties") && json_data["properties"].is_array() && 
    !json_data["properties"].empty()) {
    // 嵌套格式：合并数组为单个对象
    for (const auto& item : json_data["properties"]) {
        params.merge_patch(item);
    }
} else {
    // 扁平格式：直接使用
    params = json_data;
}
```

**实现位置**: [massage_head_manager.cpp#L394-L415](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L394-L415)

---

## 6. 支持的控制功能

### 6.1 RET能量控制（V2协议专用）
**参数名**: `ret`  
**参数范围**: `0` | `10-100`

**实现代码**:
```cpp
if (params.contains("ret")) {
    int ret_value = params["ret"].get<int>();
    auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
    
    if (!protocol_v2) {
        RCLCPP_ERROR(this->get_logger(), "当前协议不支持RET控制");
        overall_success = false;
        continue;
    }
    
    std::vector<uint8_t> cmd;
    if (ret_value == 0) {
        // 停止RET
        cmd = protocol_v2->createRETCommand(RETCommand::STOP);
    } else if (ret_value >= 10 && ret_value <= 100) {
        // 预设能量档位
        cmd = protocol_v2->createPresetCommand(PresetCommand::RET_ENERGY, ret_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // 启动RET
        auto start_cmd = protocol_v2->createRETCommand(RETCommand::START);
        enqueueControlCommand(start_cmd, "启动RET");
    }
    
    if (!cmd.empty()) {
        enqueueControlCommand(cmd, "RET控制: " + std::to_string(ret_value));
    }
}
```

**生成命令**:
- 停止: `AA 78 00 11 CC 33 C3 3C`
- 预设50档: `AA 78 07 32 CC 33 C3 3C`
- 启动: `AA 78 00 10 CC 33 C3 3C`

**实现位置**: [massage_head_manager.cpp#L446-L467](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L446-L467)

---

### 6.2 微电能量控制（V2协议专用）
**参数名**: `micro_electric`  
**参数范围**: `0` | `1-100`

**实现逻辑**:
```cpp
if (params.contains("micro_electric")) {
    int micro_value = params["micro_electric"].get<int>();
    auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
    
    if (micro_value == 0) {
        // 停止微电（调用两次Toggle）
        auto cmd1 = protocol_v2->createMicroElectricCommand(MicroElectricCommand::TOGGLE);
        auto cmd2 = protocol_v2->createMicroElectricCommand(MicroElectricCommand::TOGGLE);
        enqueueControlCommand(cmd1, "停止微电(1)");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        enqueueControlCommand(cmd2, "停止微电(2)");
    } else if (micro_value >= 1 && micro_value <= 100) {
        // 预设能量档位 + 启动
        auto preset_cmd = protocol_v2->createPresetCommand(PresetCommand::MICRO_ELECTRIC, micro_value);
        enqueueControlCommand(preset_cmd, "微电预设: " + std::to_string(micro_value));
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        auto start_cmd = protocol_v2->createMicroElectricCommand(MicroElectricCommand::TOGGLE);
        enqueueControlCommand(start_cmd, "启动微电");
    }
}
```

**生成命令**:
- 预设25档: `AA 78 09 19 CC 33 C3 3C`
- 启动/停止: `AA 78 00 3A CC 33 C3 3C`

**实现位置**: [massage_head_manager.cpp#L471-L492](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L471-L492)

---

### 6.3 负压吸力控制（V2协议专用）
**参数名**: `negative_pressure`  
**参数范围**: `0` | `1-16`

**实现逻辑**:
```cpp
if (params.contains("negative_pressure")) {
    int pressure_value = params["negative_pressure"].get<int>();
    auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
    
    if (pressure_value == 0) {
        // 停止负压（调用两次Toggle）
        auto cmd1 = protocol_v2->createNegativePressureCommand(NegativePressureCommand::TOGGLE);
        auto cmd2 = protocol_v2->createNegativePressureCommand(NegativePressureCommand::TOGGLE);
        enqueueControlCommand(cmd1, "停止负压(1)");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        enqueueControlCommand(cmd2, "停止负压(2)");
    } else if (pressure_value >= 1 && pressure_value <= 16) {
        // 预设吸力档位 + 启动
        auto preset_cmd = protocol_v2->createPresetCommand(PresetCommand::NEGATIVE_SUCTION, pressure_value);
        enqueueControlCommand(preset_cmd, "负压预设: " + std::to_string(pressure_value));
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        auto start_cmd = protocol_v2->createNegativePressureCommand(NegativePressureCommand::TOGGLE);
        enqueueControlCommand(start_cmd, "启动负压");
    }
}
```

**生成命令**:
- 预设8档: `AA 78 05 08 CC 33 C3 3C`
- 启动/停止: `AA 78 00 6A CC 33 C3 3C`

---

### 6.4 温度控制
**参数名**: `temperature`  
**参数范围**: 
- V2协议: `0` | `10-75`（档位）
- V1协议: `0` | `30-60`（°C实际温度）

**实现逻辑**:
```cpp
if (params.contains("temperature")) {
    float temp_value = params["temperature"].get<float>();
    
    if (temp_value == 0.0f) {
        // 关闭发热片
        sendTempCommand(0.0f);
    } else {
        auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
        if (protocol_v2) {
            // V2协议：使用PresetCommand预设档位
            if (temp_value >= 10 && temp_value <= 75) {
                auto cmd = protocol_v2->createPresetCommand(
                    PresetCommand::TEMPERATURE, 
                    temp_value
                );
                enqueueControlCommand(cmd, "温度预设: " + std::to_string(temp_value));
            }
        } else {
            // V1协议：直接设置温度值
            if (temp_value >= 30.0f && temp_value <= 60.0f) {
                sendTempCommand(temp_value);
            }
        }
    }
}
```

**生成命令（V2协议）**:
- 预设40档: `AA 78 0D 28 CC 33 C3 3C`

**实现位置**: [massage_head_manager.cpp#L521-L549](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L521-L549)

---

### 6.5 电机控制
**参数名**: `motor`  
**参数范围**: `0` | `1` | `2`
- `0`: 停止
- `1`: 正转
- `2`: 反转

**实现逻辑**:
```cpp
if (params.contains("motor")) {
    int motor_state = params["motor"].get<int>();
    
    if (motor_state >= 0 && motor_state <= 2) {
        sendMotorCommand(static_cast<uint8_t>(motor_state));
    }
}
```

**生成命令（V2协议）**:
- 正转: `AA 78 00 01 CC 33 C3 3C`
- 反转: `AA 78 00 02 CC 33 C3 3C`
- 停止: `AA 78 00 03 CC 33 C3 3C`

**实现位置**: [massage_head_manager.cpp#L553-L565](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L553-L565)

---

### 6.6 冲击波控制（V2协议专用）
#### 6.6.1 冲击波脉冲控制
**参数名**: `shock_wave`  
**参数范围**: `0` | 正整数（脉冲数）

```cpp
if (params.contains("shock_wave")) {
    int shock_wave_value = params["shock_wave"].get<int>();
    auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
    
    if (shock_wave_value > 0) {
        // 启动冲击波
        auto cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
        enqueueControlCommand(cmd, "冲击波启动");
    } else {
        // 停止冲击波
        auto cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
        enqueueControlCommand(cmd, "冲击波停止");
    }
}
```

#### 6.6.2 冲击波频率控制
**参数名**: `frequency`  
**参数范围**: `1-21` (Hz)

```cpp
if (params.contains("frequency")) {
    int freq_value = params["frequency"].get<int>();
    auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
    
    if (freq_value >= 1 && freq_value <= 21) {
        auto cmd = protocol_v2->createPresetCommand(
            PresetCommand::SHOCK_WAVE_FREQ, 
            freq_value
        );
        enqueueControlCommand(cmd, "冲击波频率预设: " + std::to_string(freq_value));
    }
}
```

#### 6.6.3 冲击波能量控制
**参数名**: `energy`  
**参数范围**: `1-16` (档位)

```cpp
if (params.contains("energy")) {
    int energy_value = params["energy"].get<int>();
    auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
    
    if (energy_value >= 1 && energy_value <= 16) {
        auto cmd = protocol_v2->createPresetCommand(
            PresetCommand::SHOCK_WAVE_ENERGY, 
            energy_value
        );
        enqueueControlCommand(cmd, "冲击波能量预设: " + std::to_string(energy_value));
    }
}
```

**生成命令**:
- 启动/停止: `AA 78 00 5A CC 33 C3 3C`
- 频率10Hz: `AA 78 12 0A CC 33 C3 3C`
- 能量8档: `AA 78 13 08 CC 33 C3 3C`

---

## 7. 命令队列系统

### 7.1 队列架构
```
入队 → 优先队列 → 处理线程 → 串口发送
```

**优先级定义**:
```cpp
enum class CommandPriority {
    CONTROL_HIGH = 0,    // 控制命令（最高优先级）
    STATUS_READ = 1,     // 状态查询
    IDENTIFY = 2         // 设备识别
};
```

### 7.2 入队接口
```cpp
bool enqueueControlCommand(
    const std::vector<uint8_t>& data,     // 命令数据
    const std::string& description,        // 命令描述
    std::function<void(bool)> callback     // 执行回调（可选）
);
```

**实现位置**: [massage_head_manager.cpp#L2426-L2463](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L2426-L2463)

### 7.3 处理线程
```cpp
void MassageHeadManageNode::commandProcessorLoop()
{
    while (command_processor_running_.load() && rclcpp::ok()) {
        CommandItem item;
        
        {
            std::unique_lock<std::mutex> lock(command_queue_mutex_);
            command_queue_cv_.wait(lock, [this] {
                return !command_queue_.empty() || !command_processor_running_.load();
            });
            
            if (!command_processor_running_.load()) break;
            
            item = command_queue_.top();
            command_queue_.pop();
        }
        
        // 执行命令
        bool success = sendSerialData(item.data);
        
        // 执行回调
        if (item.callback) {
            item.callback(success);
        }
        
        // 命令间隔50ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
```

**实现位置**: [massage_head_manager.cpp#L2359-L2424](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L2359-L2424)

---

## 8. 串口发送机制

### 8.1 发送接口
```cpp
bool MassageHeadManageNode::sendSerialData(const std::vector<uint8_t>& data)
{
    // 加锁保护串口写操作
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    
    if (!serial_driver_ || !serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "串口未打开，无法发送数据");
        return false;
    }

    try {
        // 发送数据
        serial_driver_->port()->send(data);
        
        // 打印发送的命令（十六进制）
        RCLCPP_DEBUG(this->get_logger(), "发送命令: %s", 
                    utils::bytesToHexString(data).c_str());
        
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "串口发送异常: %s", ex.what());
        return false;
    }
}
```

**实现位置**: [massage_head_manager.cpp#L1759-L1809](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L1759-L1809)

### 8.2 线程安全
- 使用 `serial_write_mutex_` 保护串口写操作
- 防止多个定时器和控制命令并发发送导致数据冲突

---

## 9. 协议层实现

### 9.1 协议接口（IProtocol）
```cpp
class IProtocol {
public:
    virtual BaseParseResult parsePacket(const std::vector<uint8_t> &data) = 0;
    virtual std::vector<uint8_t> createMotorControlCommand(uint8_t motor_state) = 0;
    virtual std::vector<uint8_t> createLedControlCommand(bool led_on) = 0;
    virtual InternalStatus getCurrentStatus() = 0;
    // ...
};
```

### 9.2 V2协议实现（ProtocolV2）
**关键方法**:
```cpp
// 预设命令生成（统一接口）
std::vector<uint8_t> ProtocolV2::createPresetCommand(PresetCommand cmd, uint8_t value)
{
    std::vector<uint8_t> packet = {
        0xAA, 0x78,                    // 帧头
        static_cast<uint8_t>(cmd),     // 功能码
        value,                          // 预设值
        0xCC, 0x33, 0xC3, 0x3C         // 帧尾
    };
    return packet;
}

// RET控制命令
std::vector<uint8_t> ProtocolV2::createRETCommand(RETCommand cmd)
{
    return {0xAA, 0x78, 0x00, static_cast<uint8_t>(cmd), 0xCC, 0x33, 0xC3, 0x3C};
}

// 微电控制命令
std::vector<uint8_t> ProtocolV2::createMicroElectricCommand(MicroElectricCommand cmd)
{
    return {0xAA, 0x78, 0x00, static_cast<uint8_t>(cmd), 0xCC, 0x33, 0xC3, 0x3C};
}

// 负压控制命令
std::vector<uint8_t> ProtocolV2::createNegativePressureCommand(NegativePressureCommand cmd)
{
    return {0xAA, 0x78, 0x00, static_cast<uint8_t>(cmd), 0xCC, 0x33, 0xC3, 0x3C};
}

// 冲击波控制命令
std::vector<uint8_t> ProtocolV2::createShockWaveCommand(ShockWaveCommand cmd)
{
    return {0xAA, 0x78, 0x00, static_cast<uint8_t>(cmd), 0xCC, 0x33, 0xC3, 0x3C};
}
```

**文件位置**: [protocol_v2.hpp](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/include/massage_head_manager/protocol_v2.hpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Finclude%2Fmassage_head_manager%2Fprotocol_v2.hpp%22%2C%22ref%22%3A%22%22%7D#L1)

---

## 10. 错误处理与日志

### 10.1 异常捕获
```cpp
try {
    // JSON解析
    nlohmann::json params = nlohmann::json::parse(request->command_params);
    // ...
} catch (const nlohmann::json::exception& e) {
    response->success = false;
    response->result = "JSON 解析错误: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "JSON 解析错误: %s", e.what());
} catch (const std::exception& e) {
    response->success = false;
    response->result = "处理错误: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "处理错误: %s", e.what());
}
```

### 10.2 日志级别
- **DEBUG**: 命令发送详情、数据包内容
- **INFO**: 服务请求、命令执行
- **WARN**: 序列号不匹配、参数范围错误
- **ERROR**: JSON解析失败、串口异常

---

## 11. 使用示例

### 11.1 Python客户端示例
```python
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import MassageHeadControl
import json

class MassageHeadClient(Node):
    def __init__(self):
        super().__init__('massage_head_client')
        self.client = self.create_client(MassageHeadControl, 'massage_head_control')
        
    def send_control_command(self, serial_number, params):
        request = MassageHeadControl.Request()
        request.serial_number = serial_number
        request.command_params = json.dumps(params)
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# 使用示例
client = MassageHeadClient()

# 1. 启动RET（50档）+ 微电（25档）
result = client.send_control_command(
    "5A-A5-06-83-10-01-00-01-06",
    {"ret": 50, "micro_electric": 25}
)

# 2. 设置温度40档
result = client.send_control_command(
    "5A-A5-06-83-10-01-00-01-06",
    {"temperature": 40}
)

# 3. 启动电机正转
result = client.send_control_command(
    "5A-A5-06-83-10-01-00-01-06",
    {"motor": 1}
)

# 4. 综合控制
result = client.send_control_command(
    "5A-A5-06-83-10-01-00-01-06",
    {
        "ret": 50,
        "micro_electric": 25,
        "negative_pressure": 8,
        "temperature": 40
    }
)
```

### 11.2 C++客户端示例
```cpp
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/massage_head_control.hpp>
#include <nlohmann/json.hpp>

auto client = node->create_client<robot_interfaces::srv::MassageHeadControl>("massage_head_control");

auto request = std::make_shared<robot_interfaces::srv::MassageHeadControl::Request>();
request->serial_number = "5A-A5-06-83-10-01-00-01-06";

nlohmann::json params = {
    {"ret", 50},
    {"micro_electric", 25},
    {"temperature", 40}
};
request->command_params = params.dump();

auto future = client->async_send_request(request);
auto response = future.get();

if (response->success) {
    RCLCPP_INFO(node->get_logger(), "控制成功: %s", response->result.c_str());
} else {
    RCLCPP_ERROR(node->get_logger(), "控制失败: %s", response->result.c_str());
}
```

---

## 12. 性能优化

### 12.1 数据库缓存
- 启动时全量加载数据到内存
- 查询优先从缓存获取，减少数据库I/O
- 缓存结构使用 `std::map` 实现O(log n)查询

### 12.2 命令队列
- 异步处理命令，避免阻塞服务响应
- 优先队列保证控制命令优先执行
- 队列大小限制防止内存溢出（最大50条）

### 12.3 线程安全
- 串口写操作使用互斥锁保护
- 数据库访问使用互斥锁保护
- 命令队列使用条件变量同步

---

## 13. 总结

### 13.1 关键特性
1. ✅ **数据库驱动**: 通过序列号查询数据库获取按摩头配置
2. ✅ **序列号验证**: 确保控制命令发送到正确的在线设备
3. ✅ **协议分发**: 自动识别协议版本（V1/V2）并分发命令
4. ✅ **命令队列**: 异步处理命令，保证执行顺序和间隔
5. ✅ **线程安全**: 多线程环境下保证数据一致性
6. ✅ **错误处理**: 完整的异常捕获和日志记录

### 13.2 数据流向
```
JSON请求 
  → 序列号格式转换（连字符→空格）
  → 在线设备验证（current_serial_number_）
  → JSON参数解析（nlohmann::json）
  → 数据库查询（可选，用于switch服务）
  → 协议命令生成（ProtocolV2）
  → 命令队列入队（enqueueControlCommand）
  → 串口发送（sendSerialData）
  → 响应返回
```

### 13.3 核心文件
- **主实现**: [massage_head_manager.cpp#L389-L640](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L389-L640)
- **数据库管理**: [database_manager.cpp](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/database_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L1)
- **V2协议**: [protocol_v2.hpp](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/include/massage_head_manager/protocol_v2.hpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Finclude%2Fmassage_head_manager%2Fprotocol_v2.hpp%22%2C%22ref%22%3A%22%22%7D#L1) / [protocol_v2.cpp](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/protocol_v2.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fprotocol_v2.cpp%22%2C%22ref%22%3A%22%22%7D#L1)

---

**文档版本**: 1.0  
**最后更新**: 2026-01-30  
**作者**: wlzc
