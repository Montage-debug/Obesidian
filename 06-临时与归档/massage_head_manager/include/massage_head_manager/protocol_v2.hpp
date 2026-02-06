/**
 * @file protocol_v2.hpp
 * @brief 机械臂手柄控制协议V2实现（继承自 IProtocol）
 * 
 * @details
 * ==================== 协议架构说明 ====================
 * 
 * 【接收帧格式】（手柄→上位机）
 * 5A A5 [长度] [功能码] [数据]
 * 示例: 5A A5 06 83 10 01 00 01 XX (手柄识别帧)
 * 
 * 【下发帧格式】（上位机→手柄）
 * AA 78 [参数1] [参数2] [命令码] CC 33 C3 3C
 * 长度: 固定8字节
 * 示例: AA 78 00 10 CC 33 C3 3C (启动RET)
 * 
 * ==================== 五种手柄 ====================
 * 1. 负压手柄   (ID=0x06): RET+微电+负压
 * 2. 波纹平手柄 (ID=0x07): RET+微电+发热
 * 3. 旋转手柄   (ID=0x05): RET+微电+电机
 * 4. 指疗手柄   (ID=0x04): RET+微电
 * 5. 冲击波手柄 (ID=0x08): 冲击波控制
 * 
 * ==================== 协议特点 ====================
 * - 设备主动上报识别码（无需下发识别命令）
 * - 接收帧头: 5A A5
 * - 发送帧头: AA 78
 * - 发送帧尾: CC 33 C3 3C
 * - 命令长度: 固定8字节
 * 
 * @author wlzc
 * @date 2026-01-28
 */

#ifndef PROTOCOL_V2_HPP
#define PROTOCOL_V2_HPP

#pragma once

#include "protocol_interface.hpp"
#include "protocol_utils.hpp"
#include "database_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <map>

namespace massage_head_manager {

// 前向声明（CommandType 在 protocol_v1.hpp 中定义）
enum class CommandType;

// ============================================================================
// 枚举定义
// ============================================================================

/**
 * @enum HandleType
 * @brief 手柄类型枚举
 */
enum class HandleType : uint8_t
{
    UNKNOWN = 0x00,         // 未知手柄
    FINGER_THERAPY = 0x04,  // 指疗手柄
    ROTATION = 0x05,        // 旋转手柄
    NEGATIVE_PRESSURE = 0x06, // 负压手柄
    WAVE_FLAT = 0x07,       // 波纹平手柄
    SHOCK_WAVE = 0x08       // 冲击波手柄
};

/**
 * @enum RETCommand
 * @brief RET控制命令（所有支持RET的手柄通用）
 */
enum class RETCommand : uint8_t
{
    START = 0x10,           // 启动RET
    STOP = 0x11,            // 停止RET
    TIME_INCREASE = 0x08,   // RET时间加
    TIME_DECREASE = 0x09,   // RET时间减
    ENERGY_INCREASE = 0x03, // RET能量加
    ENERGY_DECREASE = 0x04  // RET能量减
};

/**
 * @enum MicroElectricCommand
 * @brief 微电控制命令（所有支持微电的手柄通用）
 */
enum class MicroElectricCommand : uint8_t
{
    TOGGLE = 0x3A,          // 微电启停
    TIME_INCREASE = 0x37,   // 微电时间加
    TIME_DECREASE = 0x36,   // 微电时间减
    ENERGY_INCREASE = 0x3B, // 微电能量加
    ENERGY_DECREASE = 0x3C  // 微电能量减
};

/**
 * @enum NegativePressureCommand
 * @brief 负压手柄专属命令
 */
enum class NegativePressureCommand : uint8_t
{
    TOGGLE = 0x6A,          // 负压启停
    SUCTION_INCREASE = 0x69,// 负压吸力加
    SUCTION_DECREASE = 0x68 // 负压吸力减
};

/**
 * @enum RotationMotorCommand
 * @brief 旋转手柄电机控制命令
 */
enum class RotationMotorCommand : uint8_t
{
    FORWARD = 0x01,         // 电机正转
    REVERSE = 0x02,         // 电机反转
    STOP = 0x03             // 电机停止
};

/**
 * @enum ShockWaveCommand
 * @brief 冲击波手柄专属命令
 */
enum class ShockWaveCommand : uint8_t
{
    TOGGLE = 0x5A,          // 冲击波启停
    FREQ_INCREASE = 0x56,   // 频率加
    FREQ_DECREASE = 0x55,   // 频率减
    ENERGY_INCREASE = 0x58, // 能量加
    ENERGY_DECREASE = 0x57, // 能量减
    COUNT_RESET = 0x54      // 计数清零
};

/**
 * @enum PresetCommand
 * @brief 预设命令功能码（用于绝对值设置）
 */
enum class PresetCommand : uint8_t
{
    NEGATIVE_SUCTION = 0x05,    // 负压吸力预设
    RET_ENERGY = 0x07,          // RET能量预设
    MICRO_ELECTRIC = 0x09,      // 微电能量预设
    TEMPERATURE = 0x0D,         // 温度预设
    SHOCK_WAVE_FREQ = 0x12,     // 冲击波频率预设
    SHOCK_WAVE_ENERGY = 0x13    // 冲击波能量预设
};

/**
 * @enum TemperatureCommand
 * @brief 温度控制命令
 */
enum class TemperatureCommand : uint8_t
{
    PRESET = 0x0D,      // 温度预设
    INCREASE = 0x3D,    // 温度加
    DECREASE = 0x3E     // 温度减
};

/**
 * @enum StatusReadCommand
 * @brief 状态读取功能码（接收帧使用）
 */
enum class StatusReadCommand : uint8_t
{
    TEMPERATURE = 0x05,     // 读取温度
    RF_DETECT = 0x08,       // 射频检测
    MICRO_DETECT = 0x06,    // 微电检测
    USAGE_TIME = 0x09       // 使用时间
};


/**
 * @class ProtocolV2
 * @brief 机械臂手柄控制协议实现类
 * 
 * @details
 * 实现IProtocol接口，提供机械臂手柄协议的完整功能：
 * - 手柄自动识别（被动接收）
 * - RET控制（射频能量传输）
 * - 微电控制
 * - 负压控制（负压手柄专用）
 * - 电机控制（旋转手柄专用）
 * - 冲击波控制（冲击波手柄专用）
 * - 温度/状态读取
 * - 设备在线检测
 * 
 * 协议特性：
 * - 无需主动下发识别命令（设备自动上报）
 * - 固定8字节命令格式
 * - 支持多种手柄类型
 */
class ProtocolV2 : public IProtocol
{
public:
    /**
     * @brief 构造函数
     * @param logger ROS2日志记录器
     * @param clock ROS2时钟对象
     * @param db_manager 数据库管理器（可选）
     */
    explicit ProtocolV2(const rclcpp::Logger &logger, 
                       rclcpp::Clock::SharedPtr clock,
                       std::shared_ptr<DatabaseManager> db_manager = nullptr);

    // ============================================================================
    // IProtocol 接口实现
    // ============================================================================
    
    /**
     * @brief 解析接收到的数据包（手柄→上位机）
     * @param data 原始字节数据（格式: 5A A5 [长度] [功能码] [数据]）
     * @return BaseParseResult 解析结果
     */
    BaseParseResult parsePacket(const std::vector<uint8_t> &data) override;
    
    /**
     * @brief 打印解析结果
     * @param result 解析结果
     */
    void printParseResult(const BaseParseResult &result) override;

    /**
     * @brief 创建设备识别命令（V2协议不使用，设备自动上报）
     * @param handle_id 手柄ID（保留参数，兼容接口）
     * @return 空命令（V2协议设备自动识别）
     */
    std::vector<uint8_t> createIdentifyCommand(uint8_t handle_id) override;
    
    /**
     * @brief 创建电机控制命令（旋转手柄专用）
     * @param motor_state 电机状态（0=停止，1=正转，2=反转）
     * @return 命令字节序列
     */
    std::vector<uint8_t> createMotorControlCommand(uint8_t motor_state) override;
    
    /**
     * @brief 创建LED控制命令（保留接口，V2协议暂不支持）
     * @param led_on LED开关状态
     * @return 空命令
     */
    std::vector<uint8_t> createLedControlCommand(bool led_on) override;

    /**
     * @brief 获取当前内部状态
     * @return InternalStatus 设备当前状态
     */
    InternalStatus getCurrentStatus() override;

    /**
     * @brief 创建通用命令包（兼容 massage_head_manager 调用）
     * @param cmd 命令类型枚举
     * @param value 命令参数值
     * @return 命令字节序列
     * 
     * @details 将通用命令类型映射到V2协议专用命令
     */
    std::vector<uint8_t> createCommandPacket(CommandType cmd, float value);

    // ============================================================================
    // V2协议专用命令生成方法
    // ============================================================================
    
    /**
     * @brief 创建RET控制命令
     * @param cmd RET命令类型
     * @return 命令字节序列（AA 78 00/79 [cmd] CC 33 C3 3C）
     */
    std::vector<uint8_t> createRETCommand(RETCommand cmd);
    
    /**
     * @brief 创建微电控制命令
     * @param cmd 微电命令类型
     * @return 命令字节序列
     */
    std::vector<uint8_t> createMicroElectricCommand(MicroElectricCommand cmd);
    
    /**
     * @brief 创建负压控制命令（负压手柄专用）
     * @param cmd 负压命令类型
     * @return 命令字节序列
     */
    std::vector<uint8_t> createNegativePressureCommand(NegativePressureCommand cmd);
    
    /**
     * @brief 创建旋转电机控制命令（旋转手柄专用）
     * @param cmd 电机命令类型
     * @return 命令字节序列
     */
    std::vector<uint8_t> createRotationMotorCommand(RotationMotorCommand cmd);
    
    /**
     * @brief 创建冲击波控制命令（冲击波手柄专用）
     * @param cmd 冲击波命令类型
     * @return 命令字节序列
     */
    std::vector<uint8_t> createShockWaveCommand(ShockWaveCommand cmd);
    
    /**
     * @brief 创建预设命令（通用接口，支持绝对值设置）
     * @param preset_type 预设类型枚举
     * @param value 预设值（档位/温度/频率）
     * @return 命令字节序列
     * 
     * @details 支持的预设类型：
     *  - RET能量预设 (10-100档)
     *  - 微电能量预设 (1-100档)
     *  - 负压吸力预设 (1-16档)
     *  - 温度预设 (10-75°C)
     *  - 冲击波频率预设 (1-21Hz)
     *  - 冲击波能量预设 (1-16档)
     */
    std::vector<uint8_t> createPresetCommand(PresetCommand preset_type, uint8_t value);
    
    /**
     * @brief 创建温度控制命令
     * @param cmd 温度命令类型
     * @param value 温度值（仅预设时需要，10-75°C）
     * @return 命令字节序列
     */
    std::vector<uint8_t> createTemperatureCommand(TemperatureCommand cmd, uint8_t value = 0);
    
    /**
     * @brief 创建温度读取命令
     * @return 命令字节序列（5A A5 05 82 10 05 00 XX）
     */
    std::vector<uint8_t> createTemperatureReadCommand();
    
    /**
     * @brief 创建累计用时读取命令
     * @return 命令字节序列
     */
    std::vector<uint8_t> createUsageTimeReadCommand();

    /**
     * @brief 处理串口数据 - 提取完整数据包
     * @param data 接收缓冲区数据（引用，会被修改）
     * @param callback 数据包处理回调函数
     */
    void processSerialData(std::vector<uint8_t>& data, 
                          std::function<void(const std::string&)> callback);

    // ============================================================================
    // 状态管理接口
    // ============================================================================
    
    void updateMotorState(uint8_t state);
    void updateLedState(bool on);
    void updateTempState(bool enable);
    void updateTempStateSet(float temperature);
    void resetStatus();
    void updateLastPacketTime();
    void updateHeartbeatTime();
    bool checkDeviceOnline();

    // ============================================================================
    // 离线检测接口
    // ============================================================================
    
    void incrementNoResponseCount();
    void resetNoResponseCount();
    bool shouldClearDevice();
    int getNoResponseCount() const { return no_response_count_; }
    void pauseOfflineDetection();
    void resumeOfflineDetection();
    bool isOfflineDetectionEnabled() const { return offline_detection_enabled_; }

    // ============================================================================
    // 数据库查询接口
    // ============================================================================
    
    std::string getFunctionCodeBySerialNumber(const std::string& serial_number);
    std::string getSerialNumberFromPacket(const std::vector<uint8_t>& data);

    // ============================================================================
    // 手柄类型查询
    // ============================================================================
    
    /**
     * @brief 获取当前手柄类型
     * @return HandleType 手柄类型枚举
     */
    HandleType getCurrentHandleType() const { return current_handle_type_; }
    
    /**
     * @brief 根据ID获取手柄名称
     * @param handle_id 手柄ID
     * @return 手柄名称字符串
     */
    std::string getHandleName(uint8_t handle_id) const;

private:
    // ============================================================================
    // 私有成员变量
    // ============================================================================
    
    // -------- 基础组件 --------
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<DatabaseManager> db_manager_;

    // -------- 设备状态缓存 --------
    uint8_t current_handle_id_ = 0;
    std::string current_handle_name_;
    HandleType current_handle_type_ = HandleType::UNKNOWN;
    float current_temp_set_ = 0.0f;
    float current_temp_actual_ = 0.0f;
    uint8_t current_motor_state_ = 0;
    bool current_led_state_ = false;
    uint16_t current_usage_time_ = 0;
    
    // RET和微电状态
    bool ret_enabled_ = false;
    uint8_t ret_energy_level_ = 0;
    uint8_t ret_time_setting_ = 0;
    bool micro_electric_enabled_ = false;
    uint8_t micro_energy_level_ = 0;
    uint8_t micro_time_setting_ = 0;
    
    // 专用功能状态
    bool negative_pressure_enabled_ = false;  // 负压状态
    uint8_t negative_suction_level_ = 0;      // 负压吸力级别
    bool shock_wave_enabled_ = false;         // 冲击波状态
    uint8_t shock_wave_freq_ = 0;             // 冲击波频率

    // -------- 设备在线状态 --------
    bool is_device_online_ = false;
    
    // -------- 离线检测机制 --------
    int no_response_count_ = 0;
    static constexpr int MAX_NO_RESPONSE_COUNT = 5;
    bool offline_detection_enabled_ = true;
    std::chrono::steady_clock::time_point offline_detection_pause_time_;
    static constexpr int OFFLINE_DETECTION_PAUSE_DURATION_MS = 3000;

    // -------- 状态变化跟踪 --------
    uint8_t last_motor_state_ = 0xFF;
    float last_temp_actual_ = -999.0f;
    bool last_led_state_ = false;
    uint8_t last_handle_id_ = 0xFF;

    // -------- 手柄类型映射表 --------
    static const std::map<uint8_t, std::string> handle_name_map_;

    // ============================================================================
    // 私有方法
    // ============================================================================
    
    /**
     * @brief 验证V2协议接收包有效性（5A A5格式）
     * @param data 原始字节数据
     * @return true=有效，false=无效
     */
    bool validateReceivePacket(const std::vector<uint8_t> &data);
    
    /**
     * @brief 构造V2协议下发命令（AA 78格式，固定8字节）
     * @param param1 参数1（第3字节）
     * @param param2 参数2（第4字节）
     * @param cmd_code 命令码（第5字节）
     * @return 完整命令包（AA 78 [P1] [P2] [CMD] CC 33 C3 3C）
     */
    std::vector<uint8_t> buildCommandPacket(uint8_t param1, uint8_t param2, uint8_t cmd_code);
    
    /**
     * @brief 解析手柄识别帧
     * @param data 原始数据
     * @param result 解析结果（引用输出）
     * @return true=解析成功，false=失败
     */
    bool parseHandleIdentify(const std::vector<uint8_t> &data, BaseParseResult &result);
    
    /**
     * @brief 解析温度读取帧
     * @param data 原始数据
     * @param result 解析结果（引用输出）
     * @return true=解析成功，false=失败
     */
    bool parseTemperatureRead(const std::vector<uint8_t> &data, BaseParseResult &result);
    
    /**
     * @brief 解析射频/微电检测帧
     * @param data 原始数据
     * @param result 解析结果（引用输出）
     * @return true=解析成功，false=失败
     */
    bool parseDetectionResult(const std::vector<uint8_t> &data, BaseParseResult &result);
    
    /**
     * @brief 解析使用时间帧
     * @param data 原始数据
     * @param result 解析结果（引用输出）
     * @return true=解析成功，false=失败
     */
    bool parseUsageTime(const std::vector<uint8_t> &data, BaseParseResult &result);
};

} // namespace massage_head_manager

#endif // PROTOCOL_V2_HPP
