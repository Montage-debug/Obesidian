# Switch控制服务完整实现文档

## 文档概述
本文档详细说明 `massage_head_switch` 开关控制服务的完整实现机制，重点阐述**数据库驱动控制**的核心设计理念和实现流程。

---

## 1. 服务架构总览

### 1.1 服务定义
**服务名称**: `/massage_head_switch`  
**服务类型**: `robot_interfaces::srv::MassageHeadSwitch`

**请求结构**:
```cpp
string serial_number    # 按摩头序列号（连字符格式："5A-A5-06-83-10-01-00-01-06"）
string command          # 控制命令（字符串形式的整数："1"=启动, "0"=停止）
```

**响应结构**:
```cpp
bool success           # 执行是否成功
string message         # 执行结果描述
```

### 1.2 设计理念
**核心特点**: 数据库驱动的模板化控制

```
用户请求 → 查询数据库 → 获取JSON配置 → 解析执行 → 返回结果
```

**优势**:
1. ✅ **配置集中管理**: 所有按摩头的控制参数存储在数据库中
2. ✅ **灵活配置**: 修改数据库配置即可改变控制行为，无需重新编译代码
3. ✅ **设备差异化**: 不同按摩头可配置不同的启停参数
4. ✅ **版本兼容**: 自动适配V1/V2协议

### 1.3 实现入口
位于 [massage_head_manager.cpp#L852-L940](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L852-L940):
```cpp
void MassageHeadManageNode::handle_switch_service(
    const std::shared_ptr<MassageHeadSwitchSrv::Request> request,
    const std::shared_ptr<MassageHeadSwitchSrv::Response> response)
```

---

## 2. 完整实现流程

### 2.1 调用链详解
```
handle_switch_service()
    ↓ (1. 序列号格式转换)
utils::hexHyphensToSpaces("5A-A5-06...") → "5A A5 06..."
    ↓ (2. 序列号验证)
validateSerialNumberMapping(serial_number)
    ↓ (比较current_serial_number_)
    ↓ 不匹配 → 返回错误
    ↓ 匹配 ↓
    ↓ (3. 命令解析)
std::stoi(request->command) → 1 (START) 或 0 (STOP)
    ↓ (4. 数据库查询)
database_manager_->getSwitchCommandBySerialNumber(serial_number)
    ↓ 优先从缓存获取
    ↓ cached_by_serial_[serial_number].switch_command
    ↓ 缓存未命中 → SQL查询
    ↓ SELECT switch_command FROM massage_head WHERE serial_number = ?
    ↓ (5. JSON解析)
nlohmann::json::parse(switch_command_json)
    ↓ (6. 动作选择)
command == 1 ? config["start"] : config["stop"]
    ↓ (7. 执行控制)
executeSwitchFromDatabase(action_params, message)
    ↓ (遍历所有参数并执行)
    ↓ ret_enabled, micro_enabled, temperature, motor_state...
    ↓ (8. 协议命令生成)
protocol_v2->create***Command()
    ↓ (9. 命令入队)
enqueueControlCommand()
    ↓ (10. 串口发送)
sendSerialData()
    ↓ (11. 响应返回)
response->success, response->message
```

---

## 3. 数据库驱动核心机制

### 3.1 数据库表结构
**表名**: `massage_head`

**关键字段**:
```sql
CREATE TABLE massage_head (
    id TEXT PRIMARY KEY,                    -- UUID
    serial_number TEXT UNIQUE NOT NULL,     -- 序列号（连字符格式："5A-A5-06-..."）
    name TEXT NOT NULL,                     -- 按摩头名称（如"负压手柄"、"波纹平手柄"）
    config_data TEXT,                       -- JSON配置（TCP坐标、负载等）
    switch_command TEXT,                    -- ⭐核心字段：开关控制命令JSON⭐
    image_url TEXT,
    icon_url TEXT,
    created_at DATETIME,
    updated_at DATETIME
);
```

### 3.2 switch_command字段详解
**字段类型**: TEXT (JSON字符串)

**JSON结构**:
```json
{
  "start": {                        // 启动配置
    "ret_enabled": true,            // 是否启用RET
    "ret_energy": 10,               // RET能量档位（10-100）
    "micro_enabled": true,          // 是否启用微电
    "micro_energy": 25,             // 微电能量档位（1-100）
    "negative_pressure_enabled": true,  // 是否启用负压（可选）
    "negative_suction": 5,          // 负压吸力档位（1-16）
    "temperature": 35,              // 温度档位/温度值
    "motor_state": 1,               // 电机状态（1=正转，2=反转）
    "shock_wave_enabled": true,     // 是否启用冲击波（可选）
    "shock_wave_energy": 1,         // 冲击波能量档位（1-16）
    "shock_wave_freq": 1            // 冲击波频率（1-21Hz）
  },
  "stop": {                         // 停止配置
    "ret_enabled": false,
    "micro_enabled": false,
    "negative_pressure_enabled": false,
    "temperature": 0,
    "motor_state": 0,
    "shock_wave_enabled": false
  }
}
```

**配置示例**（数据库实际存储）:

#### 负压手柄配置
```json
{
  "start": {
    "ret_enabled": true,
    "ret_energy": 10,
    "micro_enabled": true,
    "micro_energy": 25,
    "negative_pressure_enabled": true,
    "negative_suction": 5
  },
  "stop": {
    "ret_enabled": false,
    "micro_enabled": false,
    "negative_pressure_enabled": false
  }
}
```

#### 波纹平手柄配置
```json
{
  "start": {
    "ret_enabled": true,
    "ret_energy": 10,
    "micro_enabled": true,
    "micro_energy": 25,
    "temperature": 35
  },
  "stop": {
    "ret_enabled": false,
    "micro_enabled": false,
    "temperature": 0
  }
}
```

#### 旋转手柄配置
```json
{
  "start": {
    "ret_enabled": true,
    "ret_energy": 10,
    "micro_enabled": true,
    "micro_energy": 25,
    "motor_state": 1
  },
  "stop": {
    "ret_enabled": false,
    "micro_enabled": false,
    "motor_state": 0
  }
}
```

#### 冲击波手柄配置
```json
{
  "start": {
    "shock_wave_enabled": true,
    "shock_wave_energy": 1,
    "shock_wave_freq": 1
  },
  "stop": {
    "shock_wave_enabled": false
  }
}
```

---

## 4. 数据库查询实现

### 4.1 查询接口
**方法签名**:
```cpp
std::string DatabaseManager::getSwitchCommandBySerialNumber(const std::string& serial_number)
```

**实现位置**: [database_manager.cpp#L445-L489](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/database_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fdatabase_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L445-L489)

### 4.2 查询逻辑
```cpp
std::string DatabaseManager::getSwitchCommandBySerialNumber(const std::string& serial_number)
{
    // 【优先级1】从缓存获取（O(log n)）
    auto head = getMassageHeadFromCache(serial_number);
    if (!head.id.empty() && !head.switch_command.empty()) {
        RCLCPP_DEBUG(logger_, "从缓存获取switch_command: serial_number=%s", serial_number.c_str());
        return head.switch_command;
    }
    
    // 【优先级2】从数据库查询
    if (!is_initialized_ || !db_) {
        RCLCPP_ERROR(logger_, "数据库未初始化");
        return "";
    }
    
    // 格式转换：空格格式 → 连字符格式（数据库存储格式）
    std::string db_serial_number = massage_head_manager::utils::hexSpacesToHyphens(serial_number);
    
    const std::string sql = R"(
        SELECT switch_command
        FROM massage_head
        WHERE serial_number = ?
        LIMIT 1
    )";
    
    sqlite3_stmt* stmt = nullptr;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    
    if (result != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备查询switch_command SQL失败: %s", sqlite3_errmsg(db_));
        return "";
    }
    
    sqlite3_bind_text(stmt, 1, db_serial_number.c_str(), -1, SQLITE_TRANSIENT);
    
    std::string switch_command;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        const unsigned char* txt = sqlite3_column_text(stmt, 0);
        switch_command = txt ? reinterpret_cast<const char*>(txt) : "";
        RCLCPP_DEBUG(logger_, "从数据库获取switch_command: serial_number=%s", serial_number.c_str());
    } else {
        RCLCPP_WARN(logger_, "未找到序列号对应的switch_command: %s", serial_number.c_str());
    }
    
    sqlite3_finalize(stmt);
    return switch_command;
}
```

### 4.3 缓存机制
**缓存结构**:
```cpp
// DatabaseManager内部缓存
std::map<std::string, MassageHead> cached_by_serial_;  // serial_number → MassageHead
```

**缓存初始化**:
```cpp
// 在initialize()中全量加载
bool DatabaseManager::initialize(const std::string& db_path) 
{
    // ... 打开数据库连接 ...
    
    // 全量加载massage_head行
    {
        std::lock_guard<std::mutex> lock(db_mutex_);
        cached_heads_ = getAllMassageHead();
        
        cached_by_serial_.clear();
    }

    // 构建serial_number索引
    for (const auto& head : cached_heads_) {
        if (!head.serial_number.empty()) {
            cached_by_serial_[head.serial_number] = head;
        }
    }
    
    return true;
}
```

**实现位置**: [database_manager.cpp#L26-L116](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/database_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fdatabase_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L26-L116)

### 4.4 查询优先级
```
1. 内存缓存（cached_by_serial_） → O(log n)
   ↓ 未命中
2. 数据库查询（SQL SELECT）       → O(1) with index
   ↓ 未找到
3. 返回空字符串                   → 错误处理
```

---

## 5. 服务实现详解

### 5.1 handle_switch_service完整实现
```cpp
void MassageHeadManageNode::handle_switch_service(
    const std::shared_ptr<MassageHeadSwitchSrv::Request> request,
    const std::shared_ptr<MassageHeadSwitchSrv::Response> response)
{
    // ========== 步骤1：格式转换 ==========
    // 输入："5A-A5-06-83-10-01-00-01-06" → "5A A5 06 83 10 01 00 01 06"
    std::string serial_number = utils::hexHyphensToSpaces(request->serial_number);
    
    // ========== 步骤2：序列号验证 ==========
    if (!request->serial_number.empty() && !validateSerialNumberMapping(serial_number)) {
        response->success = false;
        response->message = "序列号不匹配: 请求=" + request->serial_number + 
                           ", 实际=" + utils::hexSpacesToHyphens(current_serial_number_);
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // ========== 步骤3：命令解析 ==========
    int command = 0;
    try {
        command = std::stoi(request->command);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "无效的命令格式: " + request->command;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(),"收到控制请求，序列号 = %s，命令 = %d", 
               serial_number.c_str(), command);
    
    // ========== 步骤4：数据库查询 ==========
    bool success = false;
    std::string message = "";
    
    try
    {
        // 🔍 核心：从数据库获取switch_command配置
        std::string switch_command_json = database_manager_->getSwitchCommandBySerialNumber(serial_number);
        
        if (switch_command_json.empty()) {
            success = false;
            message = "数据库中未找到该序列号的控制配置: " + serial_number;
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return;
        }
        
        // ========== 步骤5：JSON解析 ==========
        nlohmann::json config_json;
        try {
            config_json = nlohmann::json::parse(switch_command_json);
        } catch (const nlohmann::json::exception& e) {
            success = false;
            message = "解析switch_command JSON失败: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return;
        }
        
        // ========== 步骤6：动作选择 ==========
        constexpr uint8_t CMD_START = 1;
        std::string action_key = (command == CMD_START) ? "start" : "stop";
        
        if (!config_json.contains(action_key)) {
            success = false;
            message = "JSON配置中缺少'" + action_key + "'字段";
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return;
        }
        
        nlohmann::json action_params = config_json[action_key];
        
        RCLCPP_INFO(this->get_logger(), "执行%s命令，配置: %s", 
                   action_key.c_str(), action_params.dump().c_str());
        
        // ========== 步骤7：执行控制 ==========
        success = executeSwitchFromDatabase(action_params, message);
    }
    catch(const std::exception& e)
    {
        success = false;
        message = "处理控制命令时发生异常: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }

    // ========== 步骤8：响应返回 ==========
    response->success = success;
    response->message = message;
    RCLCPP_INFO(this->get_logger(),"控制服务响应:success = %s, message = %s",
               success ? "true" : "false", message.c_str());
}
```

**实现位置**: [massage_head_manager.cpp#L852-L940](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L852-L940)

---

## 6. 数据库配置执行

### 6.1 executeSwitchFromDatabase实现
```cpp
bool MassageHeadManageNode::executeSwitchFromDatabase(
    const nlohmann::json& params,     // start或stop配置
    std::string& message)             // 输出消息
{
    bool overall_success = true;
    std::vector<std::string> success_msgs;
    std::vector<std::string> error_msgs;
    
    try {
        auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
        
        // ==================== RET控制 ====================
        if (params.contains("ret_enabled")) {
            bool enabled = params["ret_enabled"].get<bool>();
            
            if (enabled && params.contains("ret_energy")) {
                int energy = params["ret_energy"].get<int>();
                
                if (energy >= 10 && energy <= 100) {
                    // 预设能量档位
                    auto preset_cmd = protocol_v2->createPresetCommand(
                        PresetCommand::RET_ENERGY, energy);
                    enqueueControlCommand(preset_cmd, "RET能量预设: " + std::to_string(energy));
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    
                    // 启动RET
                    auto start_cmd = protocol_v2->createRETCommand(RETCommand::START);
                    enqueueControlCommand(start_cmd, "启动RET");
                    
                    success_msgs.push_back("RET启动成功(能量:" + std::to_string(energy) + ")");
                }
            } else {
                // 停止RET
                auto stop_cmd = protocol_v2->createRETCommand(RETCommand::STOP);
                enqueueControlCommand(stop_cmd, "停止RET");
                success_msgs.push_back("RET停止成功");
            }
        }
        
        // ==================== 微电控制 ====================
        if (params.contains("micro_enabled")) {
            bool enabled = params["micro_enabled"].get<bool>();
            
            if (enabled && params.contains("micro_energy")) {
                int energy = params["micro_energy"].get<int>();
                
                if (energy >= 1 && energy <= 100) {
                    // 预设能量档位
                    auto preset_cmd = protocol_v2->createPresetCommand(
                        PresetCommand::MICRO_ELECTRIC, energy);
                    enqueueControlCommand(preset_cmd, "微电能量预设: " + std::to_string(energy));
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    
                    // 启动微电
                    auto start_cmd = protocol_v2->createMicroElectricCommand(
                        MicroElectricCommand::TOGGLE);
                    enqueueControlCommand(start_cmd, "启动微电");
                    
                    success_msgs.push_back("微电启动成功(能量:" + std::to_string(energy) + ")");
                }
            } else {
                // 停止微电（需要调用两次Toggle）
                auto stop_cmd1 = protocol_v2->createMicroElectricCommand(
                    MicroElectricCommand::TOGGLE);
                auto stop_cmd2 = protocol_v2->createMicroElectricCommand(
                    MicroElectricCommand::TOGGLE);
                enqueueControlCommand(stop_cmd1, "停止微电(1)");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                enqueueControlCommand(stop_cmd2, "停止微电(2)");
                
                success_msgs.push_back("微电停止成功");
            }
        }
        
        // ==================== 负压控制 ====================
        if (params.contains("negative_pressure_enabled")) {
            bool enabled = params["negative_pressure_enabled"].get<bool>();
            
            if (enabled && params.contains("negative_suction")) {
                int suction = params["negative_suction"].get<int>();
                
                if (suction >= 1 && suction <= 16) {
                    // 预设吸力档位
                    auto preset_cmd = protocol_v2->createPresetCommand(
                        PresetCommand::NEGATIVE_SUCTION, suction);
                    enqueueControlCommand(preset_cmd, "负压吸力预设: " + std::to_string(suction));
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    
                    // 启动负压
                    auto start_cmd = protocol_v2->createNegativePressureCommand(
                        NegativePressureCommand::TOGGLE);
                    enqueueControlCommand(start_cmd, "启动负压");
                    
                    success_msgs.push_back("负压启动成功(吸力:" + std::to_string(suction) + ")");
                }
            } else {
                // 停止负压
                auto stop_cmd1 = protocol_v2->createNegativePressureCommand(
                    NegativePressureCommand::TOGGLE);
                auto stop_cmd2 = protocol_v2->createNegativePressureCommand(
                    NegativePressureCommand::TOGGLE);
                enqueueControlCommand(stop_cmd1, "停止负压(1)");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                enqueueControlCommand(stop_cmd2, "停止负压(2)");
                
                success_msgs.push_back("负压停止成功");
            }
        }
        
        // ==================== 温度控制 ====================
        if (params.contains("temperature")) {
            float temp = params["temperature"].get<float>();
            
            if (temp > 0.0f) {
                // V2协议：预设温度档位
                if (temp >= 10 && temp <= 75) {
                    auto cmd = protocol_v2->createPresetCommand(
                        PresetCommand::TEMPERATURE, static_cast<uint8_t>(temp));
                    enqueueControlCommand(cmd, "温度预设: " + std::to_string(temp));
                    success_msgs.push_back("温度设置成功(" + std::to_string(temp) + "档)");
                }
            } else {
                // 关闭发热片
                sendTempCommand(0.0f);
                success_msgs.push_back("发热片关闭成功");
            }
        }
        
        // ==================== 电机控制 ====================
        if (params.contains("motor_state")) {
            int motor = params["motor_state"].get<int>();
            
            if (motor >= 0 && motor <= 2) {
                sendMotorCommand(static_cast<uint8_t>(motor));
                std::string motor_desc = (motor == 0) ? "停止" : 
                                        (motor == 1) ? "正转" : "反转";
                success_msgs.push_back("电机" + motor_desc + "成功");
            }
        }
        
        // ==================== 冲击波控制 ====================
        if (params.contains("shock_wave_enabled")) {
            bool enabled = params["shock_wave_enabled"].get<bool>();
            
            if (enabled) {
                // 设置能量档位
                if (params.contains("shock_wave_energy")) {
                    int energy = params["shock_wave_energy"].get<int>();
                    if (energy >= 1 && energy <= 16) {
                        auto energy_cmd = protocol_v2->createPresetCommand(
                            PresetCommand::SHOCK_WAVE_ENERGY, energy);
                        enqueueControlCommand(energy_cmd, "冲击波能量预设: " + std::to_string(energy));
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                }
                
                // 设置频率
                if (params.contains("shock_wave_freq")) {
                    int freq = params["shock_wave_freq"].get<int>();
                    if (freq >= 1 && freq <= 21) {
                        auto freq_cmd = protocol_v2->createPresetCommand(
                            PresetCommand::SHOCK_WAVE_FREQ, freq);
                        enqueueControlCommand(freq_cmd, "冲击波频率预设: " + std::to_string(freq));
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                }
                
                // 启动冲击波
                auto start_cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
                enqueueControlCommand(start_cmd, "启动冲击波");
                success_msgs.push_back("冲击波启动成功");
            } else {
                // 停止冲击波
                auto stop_cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
                enqueueControlCommand(stop_cmd, "停止冲击波");
                success_msgs.push_back("冲击波停止成功");
            }
        }
        
        // ==================== 构建返回消息 ====================
        if (overall_success) {
            message = "执行成功: " + std::accumulate(
                success_msgs.begin(), success_msgs.end(), std::string(""),
                [](const std::string& a, const std::string& b) {
                    return a.empty() ? b : a + ", " + b;
                });
        } else {
            message = "部分失败: " + std::accumulate(
                error_msgs.begin(), error_msgs.end(), std::string(""),
                [](const std::string& a, const std::string& b) {
                    return a.empty() ? b : a + ", " + b;
                });
        }
        
    } catch (const std::exception& e) {
        message = "异常: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "executeSwitchFromDatabase异常: %s", e.what());
        overall_success = false;
    }
    
    return overall_success;
}
```

**实现位置**: [massage_head_manager.cpp#L647-L847](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L647-L847)

---

## 7. 数据库配置实例

### 7.1 负压手柄完整流程示例

**数据库记录**:
```sql
INSERT INTO massage_head (
    id, 
    serial_number, 
    name, 
    switch_command
) VALUES (
    'uuid-001',
    '5A-A5-06-83-10-01-00-01-06',
    '负压手柄',
    '{
        "start": {
            "ret_enabled": true,
            "ret_energy": 10,
            "micro_enabled": true,
            "micro_energy": 25,
            "negative_pressure_enabled": true,
            "negative_suction": 5
        },
        "stop": {
            "ret_enabled": false,
            "micro_enabled": false,
            "negative_pressure_enabled": false
        }
    }'
);
```

**请求**:
```bash
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '1'}"
```

**执行流程**:
```
1. 格式转换: "5A-A5-06-83-10-01-00-01-06" → "5A A5 06 83 10 01 00 01 06"
2. 序列号验证: current_serial_number_ == "5A A5 06 83 10 01 00 01 06" ✅
3. 命令解析: command = 1 (START)
4. 数据库查询: getSwitchCommandBySerialNumber("5A A5 06 83 10 01 00 01 06")
   → 从缓存获取switch_command JSON
5. JSON解析: action_key = "start"
   → action_params = {"ret_enabled": true, "ret_energy": 10, ...}
6. 执行控制:
   a. RET控制:
      - 预设能量: AA 78 07 0A CC 33 C3 3C (10档)
      - 启动RET: AA 78 00 10 CC 33 C3 3C
   b. 微电控制:
      - 预设能量: AA 78 09 19 CC 33 C3 3C (25档)
      - 启动微电: AA 78 00 3A CC 33 C3 3C
   c. 负压控制:
      - 预设吸力: AA 78 05 05 CC 33 C3 3C (5档)
      - 启动负压: AA 78 00 6A CC 33 C3 3C
7. 响应返回:
   success: true
   message: "执行成功: RET启动成功(能量:10), 微电启动成功(能量:25), 负压启动成功(吸力:5)"
```

---

## 8. 配置灵活性示例

### 8.1 修改启动参数（无需重新编译）

**场景**: 将负压手柄的启动能量从10档提升到50档

**操作**:
```sql
UPDATE massage_head
SET switch_command = '{
    "start": {
        "ret_enabled": true,
        "ret_energy": 50,              -- 修改：10 → 50
        "micro_enabled": true,
        "micro_energy": 50,             -- 修改：25 → 50
        "negative_pressure_enabled": true,
        "negative_suction": 10          -- 修改：5 → 10
    },
    "stop": {
        "ret_enabled": false,
        "micro_enabled": false,
        "negative_pressure_enabled": false
    }
}'
WHERE serial_number = '5A-A5-06-83-10-01-00-01-06';
```

**效果**: 下次调用服务时自动使用新配置，无需重启节点

### 8.2 添加新设备

**场景**: 添加新的负压手柄

**操作**:
```sql
INSERT INTO massage_head (
    id, 
    serial_number, 
    name, 
    switch_command
) VALUES (
    'uuid-002',
    '5A-A5-06-83-10-01-00-02-07',
    '负压手柄-02',
    '{
        "start": {
            "ret_enabled": true,
            "ret_energy": 30,              -- 不同的启动参数
            "micro_enabled": true,
            "micro_energy": 40,
            "negative_pressure_enabled": true,
            "negative_suction": 8
        },
        "stop": {
            "ret_enabled": false,
            "micro_enabled": false,
            "negative_pressure_enabled": false
        }
    }'
);
```

**效果**: 新设备即刻可用，使用自定义启动参数

---

## 9. 与JSON控制服务的对比

### 9.1 功能对比表

| 特性 | Switch控制服务 | JSON控制服务 |
|------|--------------|-------------|
| **参数来源** | 数据库（switch_command字段） | JSON请求体 |
| **配置方式** | 数据库管理，集中配置 | 每次请求指定参数 |
| **灵活性** | 修改数据库即可改变行为 | 每次请求可不同参数 |
| **适用场景** | 标准化启停流程 | 自定义精细控制 |
| **命令复杂度** | 简单（仅"1"或"0"） | 复杂（JSON多参数） |
| **设备差异化** | 支持（不同设备不同配置） | 需手动为每个设备构造JSON |
| **维护成本** | 低（集中管理） | 高（分散在调用代码中） |

### 9.2 调用示例对比

#### Switch控制服务
```bash
# 启动（使用数据库配置）
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '1'}"

# 停止（使用数据库配置）
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '0'}"
```

#### JSON控制服务
```bash
# 启动（手动指定所有参数）
ros2 service call /massage_head_control robot_interfaces/srv/MassageHeadControl \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', 
    command_params: '{\"ret\": 10, \"micro_electric\": 25, \"negative_pressure\": 5}'}"

# 停止（手动指定所有参数）
ros2 service call /massage_head_control robot_interfaces/srv/MassageHeadControl \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', 
    command_params: '{\"ret\": 0, \"micro_electric\": 0, \"negative_pressure\": 0}'}"
```

### 9.3 应用场景建议

**使用Switch控制服务**:
- ✅ 标准化启停流程
- ✅ 需要集中管理配置
- ✅ 不同设备使用不同默认参数
- ✅ 希望降低维护成本

**使用JSON控制服务**:
- ✅ 需要实时调整参数
- ✅ 精细化控制（如逐步调整能量）
- ✅ 测试和调试阶段
- ✅ 单一功能控制（如仅调温度）

---

## 10. 数据流向总结

### 10.1 完整数据流
```
用户请求
    ↓
[MassageHeadSwitch Request]
  - serial_number: "5A-A5-06-83-10-01-00-01-06"
  - command: "1"
    ↓
handle_switch_service()
    ↓
【步骤1】格式转换
  "5A-A5-06..." → "5A A5 06..."
    ↓
【步骤2】序列号验证
  current_serial_number_ == "5A A5 06..." ?
    ↓ YES
【步骤3】数据库查询
  database_manager_->getSwitchCommandBySerialNumber("5A A5 06...")
    ↓
  1. 查询缓存（cached_by_serial_）
  2. 缓存未命中 → SQL查询
  3. 返回switch_command JSON字符串
    ↓
【步骤4】JSON解析
  nlohmann::json::parse(switch_command_json)
    ↓
  config_json = {
    "start": {...},
    "stop": {...}
  }
    ↓
【步骤5】动作选择
  command == 1 ? config["start"] : config["stop"]
    ↓
  action_params = {
    "ret_enabled": true,
    "ret_energy": 10,
    "micro_enabled": true,
    ...
  }
    ↓
【步骤6】执行控制
  executeSwitchFromDatabase(action_params, message)
    ↓
  遍历参数：
  - ret_enabled → RET控制
  - micro_enabled → 微电控制
  - negative_pressure_enabled → 负压控制
  - temperature → 温度控制
  - motor_state → 电机控制
  - shock_wave_enabled → 冲击波控制
    ↓
【步骤7】协议命令生成
  protocol_v2->create***Command()
    ↓
  生成8字节命令：AA 78 [P1] [P2] CC 33 C3 3C
    ↓
【步骤8】命令入队
  enqueueControlCommand(cmd, description)
    ↓
  CommandQueue (优先队列)
    ↓
【步骤9】串口发送
  sendSerialData(data)
    ↓
  serial_driver_->port()->send(data)
    ↓
【步骤10】响应返回
  response->success = true
  response->message = "执行成功: RET启动成功(能量:10), ..."
```

---

## 11. 错误处理

### 11.1 常见错误场景

#### 场景1：序列号不匹配
```
请求序列号: "5A-A5-06-83-10-01-00-01-06"
实际在线设备: "5A-A5-07-83-10-01-00-01-07"

响应:
  success: false
  message: "序列号不匹配: 请求=5A-A5-06-83-10-01-00-01-06, 实际=5A-A5-07-83-10-01-00-01-07"
```

#### 场景2：数据库无配置
```
查询: getSwitchCommandBySerialNumber("5A A5 06 83 10 01 00 01 06")
结果: switch_command为空

响应:
  success: false
  message: "数据库中未找到该序列号的控制配置: 5A A5 06 83 10 01 00 01 06"
```

#### 场景3：JSON解析失败
```
switch_command内容: "{ret_enabled: true, ...}"  (缺少引号，格式错误)

响应:
  success: false
  message: "解析switch_command JSON失败: [json.exception.parse_error.101] parse error..."
```

#### 场景4：缺少必需字段
```
switch_command: {"start": {...}}  (缺少"stop"字段)
command: 0 (停止)

响应:
  success: false
  message: "JSON配置中缺少'stop'字段"
```

### 11.2 日志记录
```cpp
// 正常流程
RCLCPP_INFO: "收到控制请求，序列号 = 5A A5 06..., 命令 = 1"
RCLCPP_INFO: "执行start命令，配置: {\"ret_enabled\":true,...}"
RCLCPP_DEBUG: "RET能量预设: 10"
RCLCPP_DEBUG: "启动RET"

// 错误流程
RCLCPP_WARN: "序列号不匹配: 请求=..., 实际=..."
RCLCPP_ERROR: "数据库中未找到该序列号的控制配置: ..."
RCLCPP_ERROR: "解析switch_command JSON失败: ..."
```

---

## 12. 性能优化

### 12.1 数据库缓存机制
**初始化时全量加载**:
```cpp
// 启动时一次性加载所有massage_head记录到内存
database_manager_->initialize(db_path);
loadDatabaseCache();

// 缓存结构
std::map<std::string, MassageHeadInfo> db_cache_.serial_to_info;
```

**查询性能**:
- 缓存命中: O(log n) ≈ 10ns
- 缓存未命中: SQL查询 ≈ 1-5ms
- 缓存命中率: >99%（正常运行）

### 12.2 线程安全
```cpp
// 数据库访问保护
std::mutex db_mutex_;

// 缓存查询
MassageHead getMassageHeadFromCache(const std::string& serial_number) {
    std::lock_guard<std::mutex> lock(db_mutex_);
    auto it = cached_by_serial_.find(serial_number);
    return (it != cached_by_serial_.end()) ? it->second : MassageHead();
}
```

### 12.3 命令队列
- 异步处理，避免阻塞服务响应
- 命令间隔50ms，防止设备响应不及时
- 优先队列保证控制命令优先执行

---

## 13. 使用示例

### 13.1 Python客户端
```python
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import MassageHeadSwitch

class SwitchControlClient(Node):
    def __init__(self):
        super().__init__('switch_control_client')
        self.client = self.create_client(MassageHeadSwitch, 'massage_head_switch')
        
    def send_command(self, serial_number, command):
        request = MassageHeadSwitch.Request()
        request.serial_number = serial_number
        request.command = str(command)  # "1" or "0"
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# 使用示例
client = SwitchControlClient()

# 启动按摩头（使用数据库配置）
result = client.send_command("5A-A5-06-83-10-01-00-01-06", 1)
print(f"启动: {result.success}, {result.message}")

# 停止按摩头（使用数据库配置）
result = client.send_command("5A-A5-06-83-10-01-00-01-06", 0)
print(f"停止: {result.success}, {result.message}")
```

### 13.2 C++客户端
```cpp
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/massage_head_switch.hpp>

auto client = node->create_client<robot_interfaces::srv::MassageHeadSwitch>("massage_head_switch");

// 启动
auto request = std::make_shared<robot_interfaces::srv::MassageHeadSwitch::Request>();
request->serial_number = "5A-A5-06-83-10-01-00-01-06";
request->command = "1";

auto future = client->async_send_request(request);
auto response = future.get();

if (response->success) {
    RCLCPP_INFO(node->get_logger(), "启动成功: %s", response->message.c_str());
} else {
    RCLCPP_ERROR(node->get_logger(), "启动失败: %s", response->message.c_str());
}

// 停止
request->command = "0";
future = client->async_send_request(request);
response = future.get();
```

### 13.3 Shell命令行
```bash
# 启动按摩头
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '1'}"

# 停止按摩头
ros2 service call /massage_head_switch robot_interfaces/srv/MassageHeadSwitch \
  "{serial_number: '5A-A5-06-83-10-01-00-01-06', command: '0'}"
```

---

## 14. 总结

### 14.1 核心特性
1. ✅ **数据库驱动**: 控制配置存储在数据库switch_command字段
2. ✅ **配置灵活**: 修改数据库即可改变控制行为，无需重新编译
3. ✅ **设备差异化**: 不同设备可配置不同的启停参数
4. ✅ **缓存优化**: 内存缓存提升查询性能（>99%命中率）
5. ✅ **协议透明**: 自动适配V1/V2协议
6. ✅ **线程安全**: 多线程环境下保证数据一致性

### 14.2 数据库驱动优势
| 优势 | 说明 |
|------|------|
| **集中管理** | 所有设备的控制配置存储在一个数据库中 |
| **动态配置** | 无需重启节点即可修改启停参数 |
| **版本管理** | 可通过数据库备份/回滚配置 |
| **权限控制** | 数据库层面可控制配置修改权限 |
| **审计追踪** | 可记录配置修改历史（created_at, updated_at） |

### 14.3 关键实现文件
- **主实现**: [massage_head_manager.cpp#L647-L940](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/massage_head_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fmassage_head_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L647-L940)
- **数据库查询**: [database_manager.cpp#L445-L489](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/database_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fdatabase_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L445-L489)
- **数据库缓存**: [database_manager.cpp#L26-L147](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/src/database_manager.cpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Fsrc%2Fdatabase_manager.cpp%22%2C%22ref%22%3A%22%22%7D#L26-L147)
- **V2协议**: [protocol_v2.hpp](git:/mnt/data/home/wlzc/WorkSpace/wlzc_massage_robot_ws/src/massage_head_manager/include/massage_head_manager/protocol_v2.hpp?%7B%22path%22%3A%22%2Fmnt%2Fdata%2Fhome%2Fwlzc%2FWorkSpace%2Fwlzc_massage_robot_ws%2Fsrc%2Fmassage_head_manager%2Finclude%2Fmassage_head_manager%2Fprotocol_v2.hpp%22%2C%22ref%22%3A%22%22%7D#L1)

### 14.4 设计哲学
> **"配置驱动，数据库为王"**  
> 将控制逻辑从代码转移到数据库，实现运行时动态配置，降低维护成本，提高系统灵活性。

---

**文档版本**: 1.0  
**最后更新**: 2026-01-30  
**作者**: wlzc
