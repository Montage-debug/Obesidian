/**
 * @file protocol_v2.cpp
 * @brief 机械臂手柄控制协议V2实现
 * 
 * @details
 * 实现机械臂手柄控制协议的完整功能：
 * - 手柄自动识别（被动接收识别码）
 * - RET、微电、负压、电机、冲击波控制
 * - 温度和状态读取
 * - 串口数据包解析和组装
 * 
 * @author wlzc
 * @date 2026-01-28
 */

#include "massage_head_manager/protocol_v2.hpp"
#include "massage_head_manager/protocol_v1.hpp"  // CommandType 定义
#include "massage_head_manager/protocol_utils.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <chrono>

namespace massage_head_manager {

// ============================================================================
// 静态成员初始化
// ============================================================================

const std::map<uint8_t, std::string> ProtocolV2::handle_name_map_ = {
    {0x04, "指疗手柄"},
    {0x05, "旋转手柄"},
    {0x06, "负压手柄"},
    {0x07, "波纹平手柄"},
    {0x08, "冲击波手柄"}
};

// ============================================================================
// 构造与析构
// ============================================================================

ProtocolV2::ProtocolV2(const rclcpp::Logger &logger, 
                       rclcpp::Clock::SharedPtr clock,
                       std::shared_ptr<DatabaseManager> db_manager)
: logger_(logger), clock_(clock), db_manager_(db_manager)
{
    is_device_online_ = false;
    
    if (db_manager_) {
        RCLCPP_INFO(logger_, "ProtocolV2: 数据库管理器注入成功");
    } else {
        RCLCPP_WARN(logger_, "ProtocolV2: 数据库管理器未注入");
    }
    
    RCLCPP_INFO(logger_, "ProtocolV2: 机械臂手柄协议初始化完成");
}

// ============================================================================
// 状态更新方法
// ============================================================================

void ProtocolV2::updateMotorState(uint8_t state)
{
    current_motor_state_ = state;
    RCLCPP_DEBUG(logger_, "ProtocolV2: updateMotorState -> 0x%02X", state);
}

void ProtocolV2::updateLedState(bool on)
{
    current_led_state_ = on;
    RCLCPP_DEBUG(logger_, "ProtocolV2: updateLedState -> %s", on ? "ON" : "OFF");
}

void ProtocolV2::updateTempState(bool enable)
{
    if (!enable) {
        current_temp_set_ = 0.0f;
    }
    RCLCPP_DEBUG(logger_, "ProtocolV2: updateTempState -> %s", 
                enable ? "ENABLED" : "DISABLED");
}

void ProtocolV2::updateTempStateSet(float temperature)
{
    if (temperature <= 0.0f) {
        current_temp_set_ = 0.0f;
    } else {
        current_temp_set_ = temperature;
    }
    RCLCPP_DEBUG(logger_, "ProtocolV2: updateTempStateSet -> %.1f", current_temp_set_);
}

void ProtocolV2::resetStatus()
{
    current_handle_id_ = 0;
    current_handle_name_.clear();
    current_handle_type_ = HandleType::UNKNOWN;
    current_temp_set_ = 0.0f;
    current_temp_actual_ = 0.0f;
    current_motor_state_ = 0;
    current_led_state_ = false;
    current_usage_time_ = 0;
    is_device_online_ = false;
    no_response_count_ = 0;
    
    // 重置RET和微电状态
    ret_enabled_ = false;
    ret_energy_level_ = 0;
    ret_time_setting_ = 0;
    micro_electric_enabled_ = false;
    micro_energy_level_ = 0;
    micro_time_setting_ = 0;
    
    // 重置专用功能状态
    negative_pressure_enabled_ = false;
    negative_suction_level_ = 0;
    shock_wave_enabled_ = false;
    shock_wave_freq_ = 0;
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 状态已重置");
}

void ProtocolV2::updateLastPacketTime()
{
    is_device_online_ = true;
}

void ProtocolV2::updateHeartbeatTime()
{
    is_device_online_ = true;
}

bool ProtocolV2::checkDeviceOnline()
{
    return is_device_online_;
}

// ============================================================================
// 离线检测方法
// ============================================================================

void ProtocolV2::incrementNoResponseCount()
{
    if (!offline_detection_enabled_) {
        return;
    }
    
    no_response_count_++;
    RCLCPP_DEBUG(logger_, "ProtocolV2: 未响应计数增加: %d", no_response_count_);
}

void ProtocolV2::resetNoResponseCount()
{
    if (no_response_count_ > 0) {
        RCLCPP_DEBUG(logger_, "ProtocolV2: 重置未响应计数（之前: %d）", no_response_count_);
    }
    no_response_count_ = 0;
}

bool ProtocolV2::shouldClearDevice()
{
    if (!offline_detection_enabled_) {
        return false;
    }
    
    return no_response_count_ >= MAX_NO_RESPONSE_COUNT;
}

void ProtocolV2::pauseOfflineDetection()
{
    offline_detection_enabled_ = false;
    offline_detection_pause_time_ = std::chrono::steady_clock::now();
    resetNoResponseCount();
    RCLCPP_INFO(logger_, "ProtocolV2: 离线检测已暂停");
}

void ProtocolV2::resumeOfflineDetection()
{
    offline_detection_enabled_ = true;
    resetNoResponseCount();
    RCLCPP_INFO(logger_, "ProtocolV2: 离线检测已恢复");
}

// ============================================================================
// 协议验证方法
// ============================================================================

bool ProtocolV2::validateReceivePacket(const std::vector<uint8_t> &data)
{
    // 最小长度检查：5A A5 [长度] [功能码] [数据...] = 至少5字节
    if (data.size() < 5) {
        RCLCPP_DEBUG(logger_, "ProtocolV2: 数据包太短 (%zu字节)", data.size());
        return false;
    }

    // 固定头检查：5A A5
    if (data[0] != 0x5A || data[1] != 0xA5) {
        RCLCPP_DEBUG(logger_, "ProtocolV2: 包头错误: %02X %02X", data[0], data[1]);
        return false;
    }

    // 长度字段检查
    uint8_t len = data[2];
    size_t expected_size = len + 3;  // 包头(2) + LEN(1) + 数据(len)
    
    if (data.size() < expected_size) {
        RCLCPP_DEBUG(logger_, "ProtocolV2: 长度不匹配: 期望%zu字节，实际%zu字节", 
                    expected_size, data.size());
        return false;
    }

    return true;
}

std::vector<uint8_t> ProtocolV2::buildCommandPacket(uint8_t param1, 
                                                     uint8_t param2, 
                                                     uint8_t cmd_code)
{
    // V2协议下发格式：AA 78 [P1] [P2] [CMD] CC 33 C3 3C（固定8字节）
    std::vector<uint8_t> packet = {
        0xAA, 0x78,      // 包头
        param1,          // 参数1
        param2,          // 参数2
        cmd_code,        // 命令码
        0xCC, 0x33,      // 固定结尾
        0xC3, 0x3C       // 固定结尾
    };
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 构造命令: %s", 
                utils::bytesToHexString(packet).c_str());
    
    return packet;
}

// ============================================================================
// IProtocol 接口实现 - 命令生成方法
// ============================================================================

std::vector<uint8_t> ProtocolV2::createIdentifyCommand(uint8_t handle_id)
{
    // V2协议：设备自动上报识别码，无需主动下发识别命令
    (void)handle_id;  // 避免未使用参数警告
    RCLCPP_DEBUG(logger_, "ProtocolV2: 识别命令（设备自动上报，返回空）");
    return {};
}


std::vector<uint8_t> ProtocolV2::createMotorControlCommand(uint8_t motor_state)
{
    // V2协议：电机控制命令（旋转手柄专用）
    RotationMotorCommand cmd;
    
    switch (motor_state) {
        case 0:
            cmd = RotationMotorCommand::STOP;
            RCLCPP_DEBUG(logger_, "ProtocolV2: 电机停止");
            break;
        case 1:
            cmd = RotationMotorCommand::FORWARD;
            RCLCPP_DEBUG(logger_, "ProtocolV2: 电机正转");
            break;
        case 2:
            cmd = RotationMotorCommand::REVERSE;
            RCLCPP_DEBUG(logger_, "ProtocolV2: 电机反转");
            break;
        default:
            RCLCPP_WARN(logger_, "ProtocolV2: 未知电机状态: %d", motor_state);
            return {};
    }
    
    return createRotationMotorCommand(cmd);
}

std::vector<uint8_t> ProtocolV2::createLedControlCommand(bool led_on)
{
    // V2协议暂不支持LED控制
    (void)led_on;
    RCLCPP_DEBUG(logger_, "ProtocolV2: LED控制（V2协议不支持）");
    return {};
}

std::vector<uint8_t> ProtocolV2::createCommandPacket(CommandType cmd, float value)
{
    // 将通用命令类型映射到V2协议专用命令
    RCLCPP_DEBUG(logger_, "ProtocolV2: 创建通用命令 (type=%d, value=%.1f)", 
                static_cast<int>(cmd), value);
    
    switch (cmd) {
        case CommandType::SET_MOTOR_STATE:
            // 电机控制：0=停止，1=正转，2=反转
            return createMotorControlCommand(static_cast<uint8_t>(value));
        
        case CommandType::SET_LED_STATE:
            // LED控制（V2协议不支持，但保持接口兼容）
            return createLedControlCommand(value > 0.5f);
        
        case CommandType::SET_TEMPERATURE:
            // V2协议已废弃此命令类型，使用PresetCommand::TEMPERATURE代替
            RCLCPP_WARN(logger_, "ProtocolV2: SET_TEMPERATURE已废弃，请使用PresetCommand");
            return {};
        
        default:
            RCLCPP_WARN(logger_, "ProtocolV2: 未知的命令类型: %d", static_cast<int>(cmd));
            return {};
    }
}

InternalStatus ProtocolV2::getCurrentStatus()
{
    checkDeviceOnline();

    InternalStatus status;
    status.motor_state = current_motor_state_;
    status.led_state = current_led_state_;
    status.temp_set = current_temp_set_;
    status.temp_actual = current_temp_actual_;
    status.usage_time = current_usage_time_;

    return status;
}

// ============================================================================
// V2协议专用命令生成方法
// ============================================================================

std::vector<uint8_t> ProtocolV2::createRETCommand(RETCommand cmd)
{
    uint8_t param1 = 0x00;  // 默认参数1
    uint8_t param2;
    
    switch (cmd) {
        case RETCommand::START:
            param2 = static_cast<uint8_t>(RETCommand::START);
            RCLCPP_INFO(logger_, "ProtocolV2: RET启动");
            break;
        case RETCommand::STOP:
            param2 = static_cast<uint8_t>(RETCommand::STOP);
            RCLCPP_INFO(logger_, "ProtocolV2: RET停止");
            break;
        case RETCommand::TIME_INCREASE:
        case RETCommand::TIME_DECREASE:
        case RETCommand::ENERGY_INCREASE:
        case RETCommand::ENERGY_DECREASE:
            param1 = 0x79;  // 参数调节命令使用0x79
            param2 = static_cast<uint8_t>(cmd);
            RCLCPP_DEBUG(logger_, "ProtocolV2: RET参数调节: 0x%02X", param2);
            break;
        default:
            RCLCPP_WARN(logger_, "ProtocolV2: 未知RET命令");
            return {};
    }
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createMicroElectricCommand(MicroElectricCommand cmd)
{
    uint8_t param1 = 0x79;  // 微电命令参数1固定为0x79
    uint8_t param2 = static_cast<uint8_t>(cmd);
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 微电控制命令: 0x%02X", param2);
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createNegativePressureCommand(NegativePressureCommand cmd)
{
    uint8_t param1 = 0x00;  // 负压命令参数1为0x00
    uint8_t param2 = static_cast<uint8_t>(cmd);
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 负压控制命令: 0x%02X", param2);
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createRotationMotorCommand(RotationMotorCommand cmd)
{
    uint8_t param1 = 0x00;  // 电机命令参数1为0x00
    uint8_t param2 = static_cast<uint8_t>(cmd);
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 旋转电机命令: 0x%02X", param2);
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createShockWaveCommand(ShockWaveCommand cmd)
{
    uint8_t param1 = 0x00;  // 冲击波命令参数1为0x00
    uint8_t param2 = static_cast<uint8_t>(cmd);
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 冲击波控制命令: 0x%02X", param2);
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createPresetCommand(PresetCommand preset_type, uint8_t value)
{
    uint8_t param1 = static_cast<uint8_t>(preset_type);  // 功能码
    uint8_t param2 = value;  // 预设值
    
    // 参数范围验证
    switch (preset_type) {
        case PresetCommand::RET_ENERGY:
            if (value < 10 || value > 100) {
                RCLCPP_ERROR(logger_, "ProtocolV2: RET能量超出范围: %d (应在10-100)", value);
                return {};
            }
            RCLCPP_INFO(logger_, "ProtocolV2: RET能量预设 -> %d档", value);
            break;
            
        case PresetCommand::MICRO_ELECTRIC:
            if (value < 1 || value > 100) {
                RCLCPP_ERROR(logger_, "ProtocolV2: 微电能量超出范围: %d (应在1-100)", value);
                return {};
            }
            RCLCPP_INFO(logger_, "ProtocolV2: 微电能量预设 -> %d档", value);
            break;
            
        case PresetCommand::NEGATIVE_SUCTION:
            if (value < 1 || value > 16) {
                RCLCPP_ERROR(logger_, "ProtocolV2: 负压吸力超出范围: %d (应在1-16)", value);
                return {};
            }
            RCLCPP_INFO(logger_, "ProtocolV2: 负压吸力预设 -> %d档", value);
            break;
            
        case PresetCommand::TEMPERATURE:
            if (value < 10 || value > 75) {
                RCLCPP_ERROR(logger_, "ProtocolV2: 温度超出范围: %d (应在10-75°C)", value);
                return {};
            }
            RCLCPP_INFO(logger_, "ProtocolV2: 温度预设 -> %d°C", value);
            break;
            
        case PresetCommand::SHOCK_WAVE_FREQ:
            if (value < 1 || value > 21) {
                RCLCPP_ERROR(logger_, "ProtocolV2: 冲击波频率超出范围: %d (应在1-21Hz)", value);
                return {};
            }
            RCLCPP_INFO(logger_, "ProtocolV2: 冲击波频率预设 -> %dHz", value);
            break;
            
        case PresetCommand::SHOCK_WAVE_ENERGY:
            if (value < 1 || value > 16) {
                RCLCPP_ERROR(logger_, "ProtocolV2: 冲击波能量超出范围: %d (应在1-16)", value);
                return {};
            }
            RCLCPP_INFO(logger_, "ProtocolV2: 冲击波能量预设 -> %d档", value);
            break;
            
        default:
            RCLCPP_ERROR(logger_, "ProtocolV2: 未知的预设命令类型");
            return {};
    }
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createTemperatureCommand(TemperatureCommand cmd, uint8_t value)
{
    uint8_t param1;
    uint8_t param2;
    
    switch (cmd) {
        case TemperatureCommand::PRESET:
            if (value < 10 || value > 75) {
                RCLCPP_ERROR(logger_, "ProtocolV2: 温度预设值超出范围: %d (应在10-75°C)", value);
                return {};
            }
            param1 = static_cast<uint8_t>(TemperatureCommand::PRESET);
            param2 = value;
            RCLCPP_INFO(logger_, "ProtocolV2: 温度预设 -> %d°C", value);
            break;
            
        case TemperatureCommand::INCREASE:
            param1 = 0x79;  // 温度加减使用0x79
            param2 = static_cast<uint8_t>(TemperatureCommand::INCREASE);
            RCLCPP_DEBUG(logger_, "ProtocolV2: 温度加");
            break;
            
        case TemperatureCommand::DECREASE:
            param1 = 0x79;
            param2 = static_cast<uint8_t>(TemperatureCommand::DECREASE);
            RCLCPP_DEBUG(logger_, "ProtocolV2: 温度减");
            break;
            
        default:
            RCLCPP_ERROR(logger_, "ProtocolV2: 未知的温度命令");
            return {};
    }
    
    return buildCommandPacket(param1, param2, 0xCC);
}

std::vector<uint8_t> ProtocolV2::createTemperatureReadCommand()
{
    // V2协议温度读取命令（接收帧格式，实际是查询命令）
    // 按照协议文档：读取温度 5A A5 05 82 10 05 00 XX
    // 但这是接收帧格式，需要确认是否有对应的查询命令
    // 暂时返回空，因为协议文档中没有明确的温度查询下发命令
    RCLCPP_DEBUG(logger_, "ProtocolV2: 温度读取命令（暂不支持）");
    return {};
}

std::vector<uint8_t> ProtocolV2::createUsageTimeReadCommand()
{
    // V2协议使用时间读取命令
    // 按照协议文档：使用时间 5A A5 06 83 10 09 01 ZZ YY XX
    // 这是接收帧格式，暂时返回空
    RCLCPP_DEBUG(logger_, "ProtocolV2: 累计用时读取命令（暂不支持）");
    return {};
}

// ============================================================================
// 数据包解析方法
// ============================================================================

bool ProtocolV2::parseHandleIdentify(const std::vector<uint8_t> &data, 
                                      BaseParseResult &result)
{
    // 手柄识别帧格式：5A A5 06 83 10 01 00 01 XX
    // data[0-1]: 包头 5A A5
    // data[2]:   长度 06
    // data[3]:   功能码 83
    // data[4]:   固定 10
    // data[5]:   固定 01
    // data[6]:   固定 00
    // data[7]:   固定 01
    // data[8]:   手柄ID (XX)
    
    if (data.size() < 9) {
        return false;
    }
    
    if (data[3] != 0x83 || data[4] != 0x10 || data[5] != 0x01 || 
        data[6] != 0x00 || data[7] != 0x01) {
        return false;
    }
    
    uint8_t handle_id = data[8];
    
    // 验证手柄ID有效性
    if (handle_id < 0x04 || handle_id > 0x08) {
        RCLCPP_WARN(logger_, "ProtocolV2: 无效的手柄ID: 0x%02X", handle_id);
        return false;
    }
    
    result.type = BaseParseResult::MessageType::HANDLE_INFO;
    result.handle_id = handle_id;
    result.handle_name = getHandleName(handle_id);
   
    
    current_handle_name_ = result.handle_name;
    RCLCPP_INFO(logger_, "ProtocolV2: 识别到手柄 [ID=0x%02X, 名称=%s]", 
               handle_id, result.handle_name.c_str());
    
    
    // 更新内部状态
    current_handle_id_ = handle_id;
    current_handle_type_ = static_cast<HandleType>(handle_id);
    
    return true;
}

bool ProtocolV2::parseTemperatureRead(const std::vector<uint8_t> &data, 
                                       BaseParseResult &result)
{
    // 温度读取帧格式：5A A5 05 82 10 05 00 XX
    // data[7]: 温度值 (0x0A-0x4B 对应 10-75度)
    
    if (data.size() < 8) {
        return false;
    }
    
    if (data[3] != 0x82 || data[4] != 0x10 || data[5] != 0x05) {
        return false;
    }
    
    uint8_t temp_raw = data[7];
    float temperature = static_cast<float>(temp_raw);
    
    result.type = BaseParseResult::MessageType::TEMP_READ;
    result.temp_value = temperature;
    
    current_temp_actual_ = temperature;
    
    RCLCPP_DEBUG(logger_, "ProtocolV2: 温度读取 -> %.1f°C", temperature);
    
    return true;
}

bool ProtocolV2::parseDetectionResult(const std::vector<uint8_t> &data, 
                                       BaseParseResult &result)
{
    // 射频检测：5A A5 05 82 10 08 00 00 XX (XX=00成功，01失败)
    // 微电检测：5A A5 05 82 10 06 00 00 XX (XX=00成功，01失败)
    
    if (data.size() < 9) {
        return false;
    }
    
    if (data[3] != 0x82 || data[4] != 0x10) {
        return false;
    }
    
    uint8_t func_code = data[5];
    uint8_t result_code = data[8];
    
    bool success = (result_code == 0x00);
    
    if (func_code == 0x08) {
        // 射频检测
        RCLCPP_INFO(logger_, "ProtocolV2: 射频检测 -> %s", 
                   success ? "成功" : "失败");
    } else if (func_code == 0x06) {
        // 微电检测
        RCLCPP_INFO(logger_, "ProtocolV2: 微电检测 -> %s", 
                   success ? "成功" : "失败");
    }
    
    result.type = BaseParseResult::MessageType::OTHER;
    
    return true;
}

bool ProtocolV2::parseUsageTime(const std::vector<uint8_t> &data, 
                                 BaseParseResult &result)
{
    // 使用时间帧格式：5A A5 06 83 10 09 01 ZZ YY XX
    // data[7-9]: 总分钟数（大端序）ZZ=高字节，YY=中字节，XX=低字节
    
    if (data.size() < 10) {
        return false;
    }
    
    if (data[3] != 0x83 || data[4] != 0x10 || data[5] != 0x09) {
        return false;
    }
    
    uint32_t total_minutes = (static_cast<uint32_t>(data[7]) << 16) |
                            (static_cast<uint32_t>(data[8]) << 8) |
                            static_cast<uint32_t>(data[9]);
    
    result.type = BaseParseResult::MessageType::USAGE_TIME;
    result.usage_time = static_cast<uint16_t>(total_minutes & 0xFFFF);
    
    current_usage_time_ = result.usage_time;
    
    RCLCPP_INFO(logger_, "ProtocolV2: 累计用时 -> %u 分钟 (%.1f 小时)", 
               result.usage_time, result.usage_time / 60.0f);
    
    return true;
}

BaseParseResult ProtocolV2::parsePacket(const std::vector<uint8_t> &data)
{
    BaseParseResult result;
    result.raw_data = utils::bytesToHexString(data);
    
    // 验证数据包
    if (!validateReceivePacket(data)) {
        result.success = false;
        RCLCPP_DEBUG(logger_, "ProtocolV2: 数据包验证失败");
        return result;
    }
    
    // 更新最后收到数据包的时间
    updateLastPacketTime();
    
    // 构造serial_number用于数据库查询
    result.serial_number = utils::bytesToDatabaseHexString(data);
    result.success = true;
    
    // 收到任何有效数据包，重置未响应计数器
    resetNoResponseCount();
    
    // 提取功能码
    uint8_t func_code = data[3];
    
    // 根据功能码解析
    if (func_code == 0x83 && data.size() >= 9) {
        // 0x83: 可能是识别帧或使用时间帧
        uint8_t sub_func = data[5];
        
        if (sub_func == 0x01) {
            // 手柄识别
            parseHandleIdentify(data, result);
        } else if (sub_func == 0x09) {
            // 使用时间
            parseUsageTime(data, result);
        }
    } else if (func_code == 0x82 && data.size() >= 8) {
        // 0x82: 状态读取帧
        uint8_t sub_func = data[5];
        
        if (sub_func == 0x05) {
            // 温度读取
            parseTemperatureRead(data, result);
        } else if (sub_func == 0x08 || sub_func == 0x06) {
            // 射频检测或微电检测
            parseDetectionResult(data, result);
        }
    } else {
        result.type = BaseParseResult::MessageType::OTHER;
        RCLCPP_DEBUG(logger_, "ProtocolV2: 未知功能码: 0x%02X", func_code);
    }
    
    return result;
}

// ============================================================================
// 打印解析结果
// ============================================================================

void ProtocolV2::printParseResult(const BaseParseResult &result)
{
    if (!result.success) {
        RCLCPP_DEBUG(logger_, "ProtocolV2: 解析失败 -> %s", result.raw_data.c_str());
        return;
    }

    switch (result.type) {
        case BaseParseResult::MessageType::HANDLE_INFO:
            if (result.handle_id != last_handle_id_) {
                RCLCPP_INFO(logger_, "ProtocolV2: 手柄识别 -> ID=0x%02X, 名称=%s", 
                           result.handle_id, result.handle_name.c_str());
                last_handle_id_ = result.handle_id;
            } else {
                RCLCPP_DEBUG(logger_, "ProtocolV2: 手柄识别");
            }
            break;

        case BaseParseResult::MessageType::TEMP_READ:
            if (std::abs(result.temp_value - last_temp_actual_) > 0.5f) {
                RCLCPP_INFO(logger_, "ProtocolV2: 温度读取 -> %.1f°C", result.temp_value);
                last_temp_actual_ = result.temp_value;
            } else {
                RCLCPP_DEBUG(logger_, "ProtocolV2: 温度读取 -> %.1f°C", result.temp_value);
            }
            break;

        case BaseParseResult::MessageType::USAGE_TIME:
            RCLCPP_INFO(logger_, "ProtocolV2: 累计用时 -> %u 分钟", result.usage_time);
            break;

        default:
            RCLCPP_DEBUG(logger_, "ProtocolV2: 其他消息 -> %s", result.raw_data.c_str());
            break;
    }
}


std::string ProtocolV2::getSerialNumberFromPacket(const std::vector<uint8_t>& data)
{
    return utils::bytesToDatabaseHexString(data);
}

// ============================================================================
// 串口数据处理方法
// ============================================================================

void ProtocolV2::processSerialData(std::vector<uint8_t>& data, 
                                    std::function<void(const std::string&)> callback)
{
    if (data.empty() || !callback) {
        return;
    }
    
    std::string raw_hex = utils::bytesToHexString(data);
    RCLCPP_DEBUG(logger_, "ProtocolV2: processSerialData 收到 [%zu字节]: %s", 
                data.size(), raw_hex.c_str());
    
    // V2协议接收帧格式：5A A5 [长度] [功能码] [数据...]
    // 需要根据长度字段提取完整数据包
    
    while (data.size() >= 5) {
        // 查找包头 5A A5
        size_t header_pos = 0;
        bool found_header = false;
        
        for (size_t i = 0; i <= data.size() - 2; ++i) {
            if (data[i] == 0x5A && data[i + 1] == 0xA5) {
                header_pos = i;
                found_header = true;
                break;
            }
        }
        
        if (!found_header) {
            // 没有找到包头，清空缓冲区
            RCLCPP_DEBUG(logger_, "ProtocolV2: 未找到包头，清空缓冲区");
            data.clear();
            break;
        }
        
        // 移除包头之前的数据
        if (header_pos > 0) {
            data.erase(data.begin(), data.begin() + header_pos);
            RCLCPP_DEBUG(logger_, "ProtocolV2: 移除包头前%zu字节数据", header_pos);
        }
        
        // 检查是否有足够数据读取长度字段
        if (data.size() < 3) {
            RCLCPP_DEBUG(logger_, "ProtocolV2: 数据不足，等待更多数据");
            break;
        }
        
        // 读取长度字段
        uint8_t len = data[2];
        size_t packet_size = 3 + len;  // 包头(2) + LEN(1) + 数据(len)
        
        // 检查是否有完整数据包
        if (data.size() < packet_size) {
            RCLCPP_DEBUG(logger_, "ProtocolV2: 数据包不完整，需要%zu字节，当前%zu字节", 
                        packet_size, data.size());
            break;
        }
        
        // 提取完整数据包
        std::vector<uint8_t> packet(data.begin(), data.begin() + packet_size);
        
        // 移除已处理的数据
        data.erase(data.begin(), data.begin() + packet_size);
        
        // 转换为十六进制字符串并回调
        std::string packet_hex = utils::bytesToHexString(packet);
        RCLCPP_DEBUG(logger_, "ProtocolV2: 提取完整包 [%zu字节]: %s", 
                    packet.size(), packet_hex.c_str());
        
        callback(packet_hex);
    }
}

// ============================================================================
// 手柄类型查询
// ============================================================================

std::string ProtocolV2::getHandleName(uint8_t handle_id) const
{
    auto it = handle_name_map_.find(handle_id);
    if (it != handle_name_map_.end()) {
        return it->second;
    }
    return "未知手柄";
}

} // namespace massage_head_manager
