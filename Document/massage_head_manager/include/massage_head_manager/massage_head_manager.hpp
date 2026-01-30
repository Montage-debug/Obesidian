#ifndef MASSAGE_HEAD_MANAGER_HPP_
#define MASSAGE_HEAD_MANAGER_HPP_

/**
 * @file massage_head_manager.hpp
 * @brief 按摩头管理节点头文件 - 提供按摩头设备的完整管理功能
 * @details 本文件定义了MassageHeadManageNode类，该类负责：
 *          - 串口通信管理（设备识别、数据收发、心跳维护）
 *          - 设备控制（电机、LED、温度）
 *          - 状态监控与发布
 *          - 数据库缓存与配置管理
 *          - ROS2服务接口提供
 * 
 * @note 核心功能说明：
 *       1. 串口通信层:设备自动识别、数据收发、异常重连
 *       2. 协议处理层:V1协议解析、命令生成、状态同步
 *       3. 设备控制层:电机正反转、LED开关、温度调节
 *       4. 数据管理层:数据库缓存、TCP配置、使用记录
 *       5. ROS2接口层:服务响应、状态发布、参数管理
 */

// ROS2 接口依赖
#include "robot_interfaces/msg/massage_head_state.hpp"
#include "robot_interfaces/msg/massage_head_attach_event.hpp"
#include "robot_interfaces/msg/massage_head_error_event.hpp"
#include "robot_interfaces/msg/massage_head_recovery_event.hpp"

// ROS2 服务依赖
#include "robot_interfaces/srv/massage_head_control.hpp"
#include "robot_interfaces/srv/massage_head_switch.hpp"
#include "robot_interfaces/srv/arm_set_tcp.hpp"
#include "robot_interfaces/srv/arm_set_payload.hpp"



// 内部模块依赖
#include "massage_head_manager/massage_head_base.hpp"
#include "massage_head_manager/database_manager.hpp"
#include "massage_head_manager/protocol_v1.hpp"
#include "massage_head_manager/protocol_v2.hpp"
#include "massage_head_manager/protocol_interface.hpp"
#include "massage_head_manager/protocol_utils.hpp"

// 串口驱动依赖
#include "serial_driver/serial_driver.hpp"

// ROS2 核心依赖
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

// 标准库依赖
#include <nlohmann/json.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <condition_variable>
#include <functional>

namespace massage_head_manager {

// 类型别名简化代码
using namespace drivers::serial_driver;
using MassageHeadSwitchSrv = robot_interfaces::srv::MassageHeadSwitch;
using MassageHeadControlsrv = robot_interfaces::srv::MassageHeadControl;
using ArmSetTCPsrv = robot_interfaces::srv::ArmSetTCP;
using SetPayLoadsrv = robot_interfaces::srv::ArmSetPayload;



/**
 * @class MassageHeadManageNode
 * @brief 按摩头管理节点类
 * @details 核心功能模块：
 *          1. 串口通信层：设备自动识别、数据收发、异常重连
 *          2. 协议处理层：V1协议解析、命令生成、状态同步
 *          3. 设备控制层：电机正反转、LED开关、温度调节
 *          4. 数据管理层：数据库缓存、TCP配置、使用记录
 *          5. ROS2接口层：服务响应、状态发布、参数管理
 */
class MassageHeadManageNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数 - 初始化按摩头管理节点
     * @details 执行流程：
     *          1. 初始化协议实例（ProtocolV1）
     *          2. 打开串口设备并启动读取线程
     *          3. 创建ROS2服务（manager, motor, led, temp, control）
     *          4. 创建状态发布器（massage_head_status）
     *          5. 启动定时器（状态发布、设备识别、心跳、累计用时读取）
     *          6. 初始化数据库管理器并加载缓存
     */
    MassageHeadManageNode();

    /**
     * @brief 析构函数 - 安全关闭节点并释放资源
     * @details 执行流程：
     *          1. 关闭串口连接
     *          2. 停止所有定时器
     *          3. 释放线程资源
     */
    ~MassageHeadManageNode();

    /**
     * @brief 关闭串口连接
     * @details 正确释放串口资源，清理读取线程
     *          执行顺序：
     *          1. 设置退出标志（serial_running_ = false）
     *          2. 关闭串口端口（强制释放阻塞）
     *          3. 等待读取线程结束
     *          4. 清理资源（driver, io_context, buffer）
     */
    void closeSerial();

private:
    // ============================================================================
    // 成员变量区域
    // ============================================================================

    // -------- 核心管理器 --------
    std::shared_ptr<MassageHeadManager> manager_;           ///< 按摩头管理器（协议无关层）
    std::shared_ptr<ProtocolV1> protocol_;                  ///< 通信协议实例（V1版本）
    std::unique_ptr<DatabaseManager> database_manager_;     ///< 数据库管理器

    // -------- ROS2发布器与定时器 --------
    rclcpp::Publisher<robot_interfaces::msg::MassageHeadState>::SharedPtr status_pub_;  ///< 状态发布器（包含JSON格式数据）
    rclcpp::Publisher<robot_interfaces::msg::MassageHeadAttachEvent>::SharedPtr event_pub_;// 设备安装事件发布
    rclcpp::Publisher<robot_interfaces::msg::MassageHeadErrorEvent>::SharedPtr error_pub_;// 错误事件发布
    rclcpp::Publisher<robot_interfaces::msg::MassageHeadRecoveryEvent>::SharedPtr recovery_pub_;// 串口恢复事件发布
    rclcpp::TimerBase::SharedPtr timer_;                    ///< 状态发布定时器（100ms）
    rclcpp::TimerBase::SharedPtr identify_timer_;           ///< 设备识别+温度读取定时器（300ms，识别和温度读取合并）
    rclcpp::TimerBase::SharedPtr usage_time_read_timer_;    ///< 累计用时读取定时器（60s）
    rclcpp::TimerBase::SharedPtr unified_status_read_timer_;///< 统一状态读取定时器（动态间隔）
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;          ///< 心跳定时器（3s）

    // -------- ROS2服务成员 --------
    rclcpp::Service<MassageHeadSwitchSrv>::SharedPtr massage_head_switch_;  ///< 总控服务
    rclcpp::Service<MassageHeadControlsrv>::SharedPtr control_service_; ///< JSON控制服务

    rclcpp::Client<ArmSetTCPsrv>::SharedPtr arm_set_tcp_client_; ///< Arm Set TCP客户端
    rclcpp::Client<SetPayLoadsrv>::SharedPtr arm_set_payload_client_; ///< Arm Set Payload客户端



    

    // -------- 状态管理变量 --------
    uint8_t current_motor_state_;   ///< 当前电机状态（0-停止, 1-正转, 2-反转）
    bool current_led_state_;        ///< 当前LED状态
    float current_temp_set_;        ///< 当前设置温度（°C）
    float current_temp_actual_;     ///< 当前实际温度（°C）

    uint16_t current_usage_time_;   ///< 累计用时（分钟）

    // 设备识别信息（消息字段已移除，保留内部状态）
    uint8_t current_handle_id_;     ///< 当前手柄ID
    std::string current_handle_name_; ///< 当前手柄名称
    uint8_t last_logged_handle_id_; ///< 上次记录的handle_id（用于检测新设备上线）

    // -------- TCP缓存结构 --------
    /**
     * @brief TCP坐标缓存
     * @details 避免频繁数据库查询，提升性能
     */
    struct TcpCache {
        bool is_valid;          ///< 缓存有效性
        uint8_t last_handle_id; ///< 上次缓存的handle_id
        std::map<std::string, TcpPosition> tcp_positions;  ///< 缓存的TCP坐标数据
        // std::string function_code;  ///< 缓存的功能码 - 已移除
    } tcp_cache_;

    // -------- 请求缓存结构--------
    /**
     * @brief 挂起请求缓存
     * @details 当请求的serial_number与当前设备不匹配时，缓存请求等待正确按摩头安装
     */
    struct PendingRequest {
        std::string requested_serial_number;    ///< 请求的按摩头序列号
        nlohmann::json params;                  ///< 请求参数
        std::chrono::steady_clock::time_point timestamp; ///< 请求时间戳
        std::chrono::steady_clock::time_point expiry_time; ///< 过期时间
        std::string request_id;                 ///< 请求ID
        std::shared_ptr<MassageHeadControlsrv::Response> response; ///< 响应对象异步回调
        bool is_json_request;                   ///< 是否为JSON格式请求
        std::string original_request_data;      ///< 原始请求数据
    };

    std::vector<PendingRequest> pending_requests_; ///< 挂起请求队列
    std::mutex pending_requests_mutex_;            ///< 请求队列互斥锁
    std::thread pending_requests_processor_;       ///< 请求处理线程
    std::atomic<bool> pending_processor_running_;  ///< 请求处理器运行标志
    rclcpp::TimerBase::SharedPtr pending_timer_;   ///< 挂起请求检查定时器

    // -------- 数据库缓存结构 --------
    /**
     * @brief 数据库配置缓存
     * @details 程序启动时一次性加载，避免运行时频繁查询数据库
     */
    struct DatabaseCache {
        struct MassageHeadInfo {
            std::string name;                                   ///< 按摩头名称
            std::string serial_number;                          ///< 序列号
            std::string config_data;                            ///< 配置数据JSON字符串
            std::map<std::string, TcpPosition> tcp_positions;   ///< robot_name -> {x,y,z}
        };
        // std::map<std::string, MassageHeadInfo> function_code_map;  ///< function_code -> 信息 - 已移除
        std::map<std::string, MassageHeadInfo> serial_to_info;     ///< serial_number -> 信息
        bool is_loaded;  ///< 缓存加载标志
    } db_cache_;

    std::string current_serial_number_;  ///< 当前连接的按摩头序列号

    // -------- 初始化状态跟踪 --------
    std::atomic<bool> node_shutting_down_{false};   ///< 节点关闭标志，防止异步回调访问已销毁资源
    std::mutex initialization_mutex_;               ///< 初始化状态互斥锁

    // -------- 串口通信成员 --------
    std::shared_ptr<IoContext> io_context_;         ///< IO上下文
    std::shared_ptr<SerialDriver> serial_driver_;   ///< 串口驱动
    std::thread serial_read_thread_;                ///< 串口读取线程
    std::vector<uint8_t> receive_buffer_;           ///< 接收缓冲区
    std::mutex receive_buffer_mutex_;               ///< 接收缓冲区互斥锁
    std::atomic<bool> serial_running_;              ///< 串口运行标志
    std::atomic<bool> is_reconnecting_;             ///< 重连进行中标志，防止并发重连
    std::mutex serial_init_mutex_;                  ///< 串口初始化互斥锁
    std::mutex serial_write_mutex_;                 ///< 串口写互斥锁，保护控制命令下发
    std::string device_name_;                       ///< 设备名称（/dev/tty_massage_head_manager）
    uint32_t baudrate_;                             ///< 波特率（115200）

    // -------- 统一状态读取相关 --------
    std::mutex status_read_mutex_;      ///< 状态读取互斥锁
    bool enable_unified_read_;          ///< 统一读取使能
    int unified_read_interval_;         ///< 统一读取间隔（ms）
    rclcpp::Time last_unified_read_time_;  ///< 上次统一读取时间

    std::string last_serial_data_;      ///< 上次接收的串口数据（用于变化检测）

    // 状态发布缓存
    std::string cached_serial_number_;       ///< 缓存的序列号
    std::string cached_massage_head_data_;   ///< 缓存的JSON数据

    // -------- 命令队列系统 --------
    /**
     * @brief 命令优先级枚举
     * @details 数值越小优先级越高
     */
    enum class CommandPriority : uint8_t {
        CONTROL_HIGH = 0,    ///< 用户控制命令（电机、LED、温度）- 最高优先级
        CONTROL_NORMAL = 1,  ///< 普通控制命令
        STATUS_READ = 2,     ///< 状态读取命令
        IDENTIFY = 3         ///< 识别命令 - 最低优先级
    };

    /**
     * @brief 命令项结构体
     * @details 封装待发送的串口命令
     */
    struct CommandItem {
        std::vector<uint8_t> data;              ///< 命令数据
        CommandPriority priority;                ///< 优先级
        std::string description;                 ///< 命令描述（用于日志）
        std::chrono::steady_clock::time_point enqueue_time;  ///< 入队时间
        std::function<void(bool)> callback;      ///< 完成回调（可选）
        
        // 优先级比较：优先级数值小的排在前面
        bool operator>(const CommandItem& other) const {
            return priority > other.priority;
        }
    };

    std::priority_queue<CommandItem, std::vector<CommandItem>, std::greater<CommandItem>> command_queue_;  ///< 命令优先队列
    std::mutex command_queue_mutex_;           ///< 命令队列互斥锁
    std::condition_variable command_queue_cv_; ///< 命令队列条件变量
    std::thread command_processor_thread_;     ///< 命令处理线程
    std::atomic<bool> command_processor_running_;  ///< 命令处理器运行标志
    std::chrono::milliseconds command_interval_{15};  ///< 命令发送间隔（毫秒）

    // -------- 预设功能参数 --------
    // ============================================================================
    // 功能映射与验证函数
    // ============================================================================

    /**
     * @brief 验证按摩头序列号匹配
     * @param requested_serial_number 请求的序列号
     * @return bool - true:序列号匹配成功; false:序列号不匹配
     * @details 检查请求的serial_number是否与当前连接的按摩头匹配
     *          验证流程：
     *          1. 检查当前是否有按摩头连接
     *          2. 比较请求的serial_number与当前的serial_number
     */
    bool validateSerialNumberMapping(const std::string& requested_serial_number);

    // ============================================================================
    // 挂起等待机制函数（新增）
    // ============================================================================

    /**
     * @brief 添加挂起请求到缓存队列
     * @param requested_serial_number 请求的按摩头序列号
     * @param params 请求参数
     * @param response 响应对象（用于异步回调）
     * @param is_json_request 是否为JSON格式请求
     * @param original_request_data 原始请求数据
     * @return std::string - 请求ID（用于跟踪）
     * @details 当请求的序列号与当前设备不匹配时，缓存请求等待正确按摩头安装
     */
    std::string addPendingRequest(const std::string& requested_serial_number,
                                  const nlohmann::json& params,
                                  std::shared_ptr<MassageHeadControlsrv::Response> response,
                                  bool is_json_request = false,
                                  const std::string& original_request_data = "");

    /**
     * @brief 处理挂起的请求
     * @details 检查缓存队列中的请求，如果现在有匹配的按摩头（serial_number匹配），则执行请求
     *          在以下情况调用：
     *          1. 新按摩头安装时
     *          2. 定期检查时（通过定时器）
     */
    void processPendingRequests();

    /**
     * @brief 清理过期的挂起请求
     * @details 移除超过超时时间的请求，防止队列无限增长
     *          默认超时时间：30秒
     */
    void cleanupExpiredPendingRequests();

    /**
     * @brief 获取挂起请求状态
     * @return std::vector<std::map<std::string, std::string>> - 挂起请求状态列表
     * @details 返回当前所有挂起请求的信息，用于状态查询
     */
    std::vector<std::map<std::string, std::string>> getPendingRequestsStatus();

    /**
     * @brief 挂起请求处理线程函数
     * @details 在单独线程中定期检查和处理挂起请求
     */
    void pendingRequestsProcessorLoop();

    /**
     * @brief 挂起请求检查定时器回调
     * @details 定期检查挂起请求队列，处理过期请求和可执行请求
     */
    void pendingRequestsTimerCallback();

    /**
     * @brief 发送TCP设置请求到arm_set_tcp服务
     * @param serial_number 按摩头序列号
     * @details 根据serial_number查询数据库获取TCP坐标，并发送到arm_set_tcp服务
     *          执行流程：
     *          1. 从数据库查询TCP数据（通过serial_number）
     *          2. 遍历所有机器人配置
     *          3. 构建ArmSetTCP请求（use_preset=false）
     *          4. 异步发送请求并处理响应
     */
    void sendTcpSetRequestBySerialNumber(const std::string& serial_number);

    /**
     * @brief 发送Payload设置请求到arm_set_payload服务
     * @param serial_number 按摩头序列号
     * @details 根据serial_number查询数据库获取payload质量，并发送到arm_set_payload服务
     *          执行流程：
     *          1. 从数据库查询payload数据（通过serial_number）
     *          2. 构建ArmSetPayload请求
     *          3. 异步发送请求并处理响应
     */
    void sendPayloadSetRequestBySerialNumber(const std::string& serial_number);

    // ============================================================================
    // 数据库管理函数
    // ============================================================================

    /**
     * @brief 加载数据库缓存
     * @details 从数据库加载所有按摩头配置到本地缓存
     *          执行流程：
     *          1. 从数据库获取所有按摩头配置
     *          2. 解析TCP坐标数据
     *          3. 建立serial_number映射
     *          在程序启动时调用一次，之后使用本地缓存
     */
    void loadDatabaseCache();

    /**
     * @brief 记录操作日志到数据库
     * @param operation_type 操作类型（如"START", "STOP", "PAUSE"等）
     * @param detail 操作详情
     * @param success 操作是否成功
     * @details 记录内容：
     *          - 时间戳
     *          - 按摩头ID和名称
     *          - TCP偏移量
     *          - 操作详情
     *          - 成功状态
     */
    void logOperation(const std::string& operation_type, const std::string& detail, bool success);

    // ============================================================================
    // ROS2服务回调函数
    // ============================================================================

    /**
     * @brief 处理JSON格式的按摩头控制服务
     * @param request 包含JSON字符串的服务请求（massage_head_function_params字段）
     * @param response 服务响应（result字段）
     * @details JSON格式：
     *          {
     *            "massage_head_configs": [
     *              {
     *                "serial_number": "SN20240001",
     *                "params": {
     *                  "motor": {"state": 1},
     *                  "led": {"on": true},
     *                  "temp": {"enable": true, "temp_set": 36.0}
     *                }
     *              }
     *            ]
     *          }
     *          处理流程：
     *          1. 解析JSON数据
     *          2. 验证序列号匹配
     *          3. 执行控制命令
     */
    void handle_json_control_service(
        const std::shared_ptr<MassageHeadControlsrv::Request> request,
        const std::shared_ptr<MassageHeadControlsrv::Response> response);

    /**
     * @brief 处理按摩头总控服务
     * @param request 服务请求（command字段：START/STOP/PAUSE/RESUME）
     * @param response 服务响应（success, message, 当前状态）
     * @details 支持的命令：
     *          - START: 启动按摩头（电机正转、LED开启、加热启用）
     *          - STOP: 停止按摩头（电机停止、LED关闭、加热关闭）
     *          - PAUSE: 暂停按摩头（保存状态并停止）
     *          - RESUME: 恢复按摩头（恢复到暂停前状态）
     */
    void handle_switch_service(
        const std::shared_ptr<MassageHeadSwitchSrv::Request> request,
        const std::shared_ptr<MassageHeadSwitchSrv::Response> response);

    /**
     * @brief 处理按摩头控制服务
     * @param request 服务请求（massage_head_serial_number, massage_head_function_params）
     * @param response 服务响应（result字段）
     * @details 将单个serial_number控制请求转换为JSON格式，
     *          然后调用handle_json_control_service处理
     */
    void handle_json_service(
        const std::shared_ptr<MassageHeadControlsrv::Request> request,
        const std::shared_ptr<MassageHeadControlsrv::Response> response);


    /**
     * @brief 发送电机控制命令
     * @param state 电机状态（0-停止, 1-正转, 2-反转）
     * @return bool - true:命令发送成功; false:命令发送失败
     * @details 发送后延迟100ms并主动读取状态以确认
     */
    bool sendMotorCommand(uint8_t state);

    /**
     * @brief 发送温度控制命令
     * @param temperature 目标温度（0表示关闭，>0表示启用并设置温度）
     * @return bool - true:命令发送成功; false:命令发送失败
     * @details 温度为0时关闭加热，开启时默认25°C，发送后延迟100ms并主动读取状态
     */
    bool sendTempCommand(float temperature);

    /**
     * @brief 发送LED控制命令
     * @param on LED开关（true-开启, false-关闭）
     * @return bool - true:命令发送成功; false:命令发送失败
     */
    bool sendLedCommand(bool on);

    // ============================================================================
    // JSON控制函数
    // ============================================================================

    /**
     * @brief 处理JSON格式的电机控制
     * @param state 电机状态（0-关闭, 1-顺时针, 2-逆时针）
     * @return bool - true:控制成功; false:控制失败
     */
    bool handleMotorControlJson(int state);

    /**
     * @brief 处理JSON格式的LED控制
     * @param on LED开关状态
     * @return bool - true:控制成功; false:控制失败
     */
    bool handleLedControlJson(bool on);

    /**
     * @brief 处理JSON格式的温度控制
     * @param temperature 目标温度（0表示关闭，>0表示设置温度）
     * @return bool - true:控制成功; false:控制失败
     * @details 温度为0时关闭加热，>0时启用并设置温度
     */
    bool handleTempControlJson(float temperature);

    /**
     * @brief 从JSON配置执行控制命令
     * @param params JSON格式的控制参数（扁平式结构）
     * @param message 输出消息（引用返回）
     * @return bool - true:控制成功; false:控制失败
     * @details 支持的JSON参数：
     *          - ret_enabled: RET开关
     *          - ret_energy: RET能量档位
     *          - micro_enabled: 微电开关
     *          - micro_energy: 微电能量档位
     *          - negative_pressure_enabled: 负压开关
     *          - negative_suction: 负压吸力档位
     *          - motor_state: 电机状态(0-停止, 1-正转, 2-反转)
     *          - temperature: 温度档位
     *          - shock_wave_enabled: 冲击波开关
     *          - shock_wave_energy: 冲击波能量
     *          - shock_wave_freq: 冲击波频率
     */
    bool executeSwitchFromDatabase(const nlohmann::json& params, std::string& message);

    // ============================================================================
    // 状态同步与管理函数
    // ============================================================================

    /**
     * @brief 同步硬件反馈状态到本地
     * @details 从协议层读取硬件状态并更新本地变量：
     *          - 电机状态
     *          - LED状态
     *          - 温度设置和实际值
     *          - 累计用时
     *          同时检测按摩头更换并记录日志
     */
    void syncHardwareStatusToLocal();

    /**
     * @brief 串口数据接收回调
     * @param hex_str 接收到的十六进制字符串
     * @details 数据处理流程：
     *          1. 转换十六进制字符串为字节数组
     *          2. 调用manager解析数据包
     *          3. 处理手柄识别数据包（更新serial_number）
     *          4. 同步硬件状态到本地
     *          5. 新设备上线时主动读取状态
     */
    void onSerialData(const std::string& hex_str);

    /**
     * @brief 获取本地状态
     * @return robot_interfaces::msg::MassageHeadState - 当前按摩头状态消息
     * @details 状态来源优先级：
     *          1. 设备在线时：优先使用硬件反馈状态
     *          2. 设备离线时：使用本地缓存状态
     *          3. 无协议时：使用数据库默认配置
     *          状态包含：
     *          - 在线状态、手柄ID、名称
     *          - TCP坐标（从数据库缓存加载）
     *          - 电机、LED、温度状态
     *          - 累计用时
     */
    robot_interfaces::msg::MassageHeadState getLocalStatus();

    /**
     * @brief 生成JSON格式的按摩头状态
     * @return std::string - JSON格式的状态字符串
     * @details JSON格式：
     *          {
     *            "timestamp": "2024-01-01T12:00:00Z",
     *            "device": {
     *              "serial_number": "SN20240001",
     *              "online": true
     *            },
     *            "status": {
     *              "motor": 1,
     *              "led": true,
     *              "temp": {
     *                "set": 38.0,
     *                "actual": 37.5,
     *                "enabled": true
     *              }
     *            },
     *            "usage_seconds": 3600
     *          }
     *          设备在线时返回实时状态，离线时返回空状态
     */
    std::string generateJsonStatus();

    /**
     * @brief 更新TCP缓存
     * @details 从数据库缓存加载当前设备的TCP配置数据
     */
    void updateTcpCache();


    // ============================================================================
    // 串口通信函数
    // ============================================================================

    /**
     * @brief 初始化串口设备
     * @return bool - true:初始化成功; false:初始化失败
     * @details 初始化流程：
     *          1. 检查并关闭已打开的串口
     *          2. 清理系统串口资源
     *          3. 等待设备准备就绪（500ms）
     *          4. 创建IO上下文和串口驱动
     *          5. 尝试打开串口（最多5次）
     *          6. 启动读取线程
     *          波特率：115200
     *          设备名：/dev/tty_massage_head_manager
     */
    bool initializeSerial();


    /**
     * @brief 串口读取循环线程函数
     * @details 在单独线程中持续读取串口数据：
     *          1. 检查串口状态
     *          2. 读取数据到缓冲区
     *          3. 交给协议层处理
     *          4. 异常时尝试重连
     *          5. 重连失败后等待5s再试
     */
    void serialReadLoop();

    /**
     * @brief 发送串口数据
     * @param data 要发送的字节数组
     * @return bool - true:发送成功; false:发送失败
     * @details 检查串口连接状态并发送数据
     */
    bool sendSerialData(const std::vector<uint8_t>& data);

    // ============================================================================
    // 命令队列系统函数
    // ============================================================================

    /**
     * @brief 初始化命令处理器
     * @details 启动命令处理线程，开始处理命令队列
     */
    void initCommandProcessor();

    /**
     * @brief 停止命令处理器
     * @details 安全停止命令处理线程
     */
    void stopCommandProcessor();

    /**
     * @brief 命令处理线程函数
     * @details 从命令队列中取出命令并发送，按优先级排序
     */
    void commandProcessorLoop();

    /**
     * @brief 入队控制命令（高优先级）
     * @param data 命令数据
     * @param description 命令描述
     * @param callback 完成回调（可选）
     * @return bool - 入队成功返回true
     */
    bool enqueueControlCommand(const std::vector<uint8_t>& data, 
                               const std::string& description,
                               std::function<void(bool)> callback = nullptr);

    /**
     * @brief 入队状态查询命令（低优先级）
     * @param data 命令数据
     * @param description 命令描述
     * @return bool - 入队成功返回true
     */
    bool enqueueStatusCommand(const std::vector<uint8_t>& data,
                              const std::string& description);

    /**
     * @brief 入队识别命令（最低优先级）
     * @param data 命令数据
     * @return bool - 入队成功返回true
     */
    bool enqueueIdentifyCommand(const std::vector<uint8_t>& data);

    /**
     * @brief 清空命令队列
     * @details 清空所有待发送的命令
     */
    void clearCommandQueue();

    /**
     * @brief 获取队列大小
     * @return size_t - 当前队列中的命令数量
     */
    size_t getCommandQueueSize();

    // ============================================================================
    // 预设参数配置函数
    // ============================================================================

    /**
     * @brief 初始化预设功能参数
     * @details 从参数服务器或配置文件加载预设值，未配置时使用默认值
     *          支持的参数：
     *          - start.motor_state: 开始时的电机状态 (0-2)
     *          - start.led_state: 开始时的LED状态 (true/false)
     *          - start.temperature: 开始时的温度设置
     *          - start.ret_energy: RET能量档位 (0-100)
     *          - start.micro_electric: 微电档位 (0-100)
     *          - start.negative_pressure: 负压档位 (0-16)
     *          - stop.motor_state: 停止时的电机状态
     *          - stop.led_state: 停止时的LED状态
     *          - stop.temperature: 停止时的温度设置
     */
    void initializePresetParams();

    /**
     * @brief 更新预设参数
     * @param type "start" 或 "stop"
     * @param params 参数JSON对象
     * @return bool 更新成功返回true
     */
    bool updatePresetParams(const std::string& type, const nlohmann::json& params);

    // ============================================================================
    // 状态读取函数
    // ============================================================================

    /**
     * @brief 智能调节统一读取间隔
     * @return int - 新的读取间隔（毫秒）
     * @details 根据设备运行状态动态调整读取频率：
     *          - 设备离线：5000ms（5秒）
     *          - 设备活跃（电机运行或加热启用）：1500ms（1.5秒）
     *          - 设备空闲：3000ms（3秒）
     */
    int getAdaptiveReadInterval();

    /**
     * @brief 更新统一读取定时器间隔
     * @details 根据getAdaptiveReadInterval()返回的间隔重新创建定时器
     */
    void updateUnifiedReadTimer();

    /**
     * @brief 统一设备状态读取回调函数
     * @details 合并所有设备状态的读取，确保一致性：
     *          1. 检查设备在线状态
     *          2. 读取温度状态（如果支持）
     *          3. 读取电机状态（如果支持）
     *          使用互斥锁保护，避免竞态条件
     */
    void unifiedDeviceStatusReadCallback();

    /**
     * @brief 被动读取设备状态
     * @details 在发送控制命令后调用，用于读取温度状态确认：
     *          1. 读取温度状态 (0x06)
     *          注意: 电机状态通过控制命令响应自动更新，不单独查询
     */
    void readStatusAfterCommand();

    /**
     * @brief 备份温度读取回调函数
     * @details 作为统一读取的备份机制，降低频率运行：
     *          - 加热启用时：发送温度设置命令触发反馈
     *          - 加热关闭时：发送温度读取命令
     */
    void backupTemperatureReadCallback();

    // ============================================================================
    // 协议管理函数
    // ============================================================================

    /**
     * @brief 切换通信协议
     * @param new_protocol 新的协议实例
     * @details 将协议切换为新协议并更新manager
     *          当前仅支持ProtocolV1
     */
    void switchProtocol(std::shared_ptr<IProtocol> new_protocol);
};

} // namespace massage_head_manager

#endif // MASSAGE_HEAD_MANAGER_HPP_

