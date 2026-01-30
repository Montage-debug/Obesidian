#pragma once
// 协议接口（统一契约）与通用返回结构
// IProtocol 保证：解析 -> 返回 BaseParseResult, 构造命令, 返回当前状态, 打印结果

#include <vector>
#include <string>
#include <cstdint>
#include "robot_interfaces/msg/massage_head_state.hpp"

namespace massage_head_manager
{

// 通用解析结果
struct BaseParseResult
{
    bool success = false;
    std::string raw_data;           // 原始十六进制字符串
    int handle_id = -1;
    std::string handle_name;
    std::string serial_number;      // 按摩头序列号
    float temp_value = 0.0f;        // 温度（设置/读取）
    uint8_t motor_state = 0;        // 0 停止，1 正转，2 反转
    bool led_state = false;
    uint32_t function_bits = 0;     // 协议/手柄能力位掩码
    uint16_t usage_time = 0;     // 累计用时（分钟）

    // 便于区分类型
    enum class MessageType : uint8_t {
        OTHER = 0,
        HANDLE_INFO,
        TEMP_SET,
        TEMP_READ,
        MOTOR_STATE,
        LED_STATE,
        USAGE_TIME
    } type = MessageType::OTHER;
};

// 内部状态结构：用于在协议层和管理层之间传递状态
struct InternalStatus
{
    uint8_t motor_state = 0;
    bool led_state = false;
    float temp_set = 0.0f;
    float temp_actual = 0.0f;
    uint16_t usage_time = 0;
};

// IProtocol 抽象接口：所有协议实现必须继承并实现这些方法
class IProtocol
{
public:
    virtual ~IProtocol() = default;

    // 解析原始字节包 -> 返回 BaseParseResult
    virtual BaseParseResult parsePacket(const std::vector<uint8_t> &data) = 0;

    // 打印解析结果，具体根据协议内部决定
    virtual void printParseResult(const BaseParseResult &result) = 0;

    // 构造各类命令
    virtual std::vector<uint8_t> createIdentifyCommand(uint8_t handle_id) = 0;
    virtual std::vector<uint8_t> createTemperatureSetCommand(float temperature) = 0;
    virtual std::vector<uint8_t> createMotorControlCommand(uint8_t motor_state) = 0;
    virtual std::vector<uint8_t> createLedControlCommand(bool led_on) = 0;

    // 返回当前内部状态
    virtual InternalStatus getCurrentStatus() = 0;
};

} 
