/**
 * @brief
 * 协议实现，继承自 IProtocol
 * 管理多个按摩头协议实例
 * @return
 * 指令生成（电机、LED、温度设置）
 * 数据包解析
 * 打印解析结果
 * 
 * @details
 * 要根据协议注意功能码或者命令码的  payload[]  位置
 * 
 * rclcpp::Logger 打印
 * 
*/

#ifndef PROTOCOL_V1_H
#define PROTOCOL_V1_H


#pragma once
// ProtocolV1: 将 V1 协议的全部解析、命令生成与打印放在这里。
// 实现 IProtocol，返回 BaseParseResult

#include "protocol_interface.hpp"
#include "protocol_utils.hpp"
#include "database_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <vector>
#include <memory>

namespace massage_head_manager {

//添加枚举
enum class CommandType
{
    SET_TEMPERATURE,
    SET_MOTOR_STATE,
    SET_LED_STATE
};

// 按摩头信息结构体 - 未使用，已被 DatabaseCache::MassageHeadInfo 替代
// struct MassageHeadInfo {
//     std::string serial_number;    // 完整数据包格式
//     std::string function_code;    // 功能码
//     std::string name;             // 名称
//     std::string description;      // 描述
// };



class ProtocolV1 : public IProtocol
{
public:
    
    void updateMotorState(uint8_t state);
    void updateLedState(bool on);

    void updateTempState(bool enable);
    void updateTempStateSet(float temperature);

    // 状态重置和在线检测
    void resetStatus();
    void updateLastPacketTime();
    void updateHeartbeatTime();
    bool checkDeviceOnline();
    
    // 识别包响应检测
    void incrementNoResponseCount();   // 增加未响应计数
    void resetNoResponseCount();       // 重置计数器（收到响应时）
    bool shouldClearDevice();          // 检查是否应该清除设备
    int getNoResponseCount() const { return no_response_count_; }  // 获取当前计数值
    
    // 离线检测控制接口
    void pauseOfflineDetection();      // 暂停离线检测
    void resumeOfflineDetection();     // 恢复离线检测
    bool isOfflineDetectionEnabled() const { return offline_detection_enabled_; }


    // logger 和 clock 可从调用者传入（node->get_logger(), node->get_clock()）
    explicit ProtocolV1(const rclcpp::Logger &logger, rclcpp::Clock::SharedPtr clock,
                       std::shared_ptr<DatabaseManager> db_manager = nullptr);

    // IProtocol 实现
    BaseParseResult parsePacket(const std::vector<uint8_t> &data) override;
    void printParseResult(const BaseParseResult &result) override;

    // 处理串口数据 - 提取完整数据包
    void processSerialData(std::vector<uint8_t>& data, std::function<void(const std::string&)> callback);

    std::vector<uint8_t> createIdentifyCommand(uint8_t handle_id) override;
    std::vector<uint8_t> createTemperatureSetCommand(float temperature) override;
    std::vector<uint8_t> createMotorControlCommand(uint8_t motor_state) override;
    std::vector<uint8_t> createLedControlCommand(bool led_on) override;
    std::vector<uint8_t> createUsageTimeReadCommand();
    std::vector<uint8_t> createTemperatureReadCommand();
    std::vector<uint8_t> createCommandPacket(CommandType cmd, float value);


    InternalStatus getCurrentStatus() override;

    // 功能码映射相关函数
    std::string getFunctionCodeBySerialNumber(const std::string& serial_number);
    std::string getSerialNumberFromPacket(const std::vector<uint8_t>& data);

private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<DatabaseManager> db_manager_;  // 数据库管理器


    // 缓存状态
    uint8_t current_handle_id_ = 0;
    std::string current_handle_name_;
    float current_temp_set_ = 0.0f;
    float current_temp_actual_ = 0.0f;
    uint8_t current_motor_state_ = 0;
    bool current_led_state_ = false;
    uint16_t current_usage_time_ = 0;  //累积用时读取

    // 设备在线状态标志
    bool is_device_online_ = false;
    
    // 识别包响应检测
    int no_response_count_ = 0;
    static constexpr int MAX_NO_RESPONSE_COUNT = 5;  // 无响应判定为离线
    
    // 离线检测控制
    bool offline_detection_enabled_ = true;
    std::chrono::steady_clock::time_point offline_detection_pause_time_;
    static constexpr int OFFLINE_DETECTION_PAUSE_DURATION_MS = 3000;  // 暂停3秒  

    // 状态变化跟踪
    uint8_t last_motor_state_ = 0xFF;  // 初始设为无效值
    float last_temp_actual_ = -999.0f;  // 初始设为不可能值
    bool last_led_state_ = false;
    uint8_t last_handle_id_ = 0xFF;  // 初始设为无效值
    
    // 函数
    bool validatePacketV1(const std::vector<uint8_t> &data);
    // 构造基础包（V1 定义）
    std::vector<uint8_t> buildBasePacketV1(uint8_t func_code, const std::vector<uint8_t> &payload);
};

} 


#endif
