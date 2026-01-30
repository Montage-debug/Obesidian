#include "massage_head_manager/massage_head_manager.hpp"
#include "massage_head_manager/protocol_utils.hpp"
#include "massage_head_manager/protocol_v2.hpp"
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <numeric>
#include <license_manager/license_validator.hpp>

using namespace std::chrono_literals;
using namespace massage_head_manager;


MassageHeadManageNode::MassageHeadManageNode()
: Node("massage_head_manage"),
  current_motor_state_(0),
  current_led_state_(false),
  current_temp_set_(0.0f),
  current_temp_actual_(0.0f),
  current_usage_time_(0),
  current_handle_id_(0),
  current_handle_name_("Unknown"),
  last_logged_handle_id_(0),
  pending_processor_running_(false),
  serial_running_(false),
  is_reconnecting_(false),
  device_name_("/dev/tty_massage_head_manager"),
  baudrate_(115200),
  enable_unified_read_(true),
  unified_read_interval_(2500),
  last_serial_data_(""),
  command_processor_running_(false)
{
    RCLCPP_INFO(this->get_logger(), "启动按摩头管理节点");

    //初始化数据库管理器
    database_manager_ = std::make_unique<DatabaseManager>(this->get_logger());

    // 从环境变量获取数据库路径
    const char* env_db_path = std::getenv("MR_DATABASE_PATH");
    std::string db_path;
    
    if (env_db_path && env_db_path[0] != '\0') {
        db_path = env_db_path;
    } 
    if(!database_manager_->initialize(db_path))
    {
        RCLCPP_ERROR(this->get_logger(),"数据库初始化失败");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"数据库初始化成功");
        // 加载数据库缓存到本地
        loadDatabaseCache();
    }

    // 初始化协议实例
    std::shared_ptr<DatabaseManager> db_shared = std::shared_ptr<DatabaseManager>(
        database_manager_.get(), 
        [](DatabaseManager*){}
    );
    
    protocol_ = std::make_shared<ProtocolV1>(this->get_logger(), this->get_clock(), db_shared);
    
    // manager 使用默认 ProtocolV1,可通过 setActiveProtocol 注入其它协议
    manager_ = std::make_shared<MassageHeadManager>(this->get_clock());
    
    // 设置按摩头管理器使用的通信协议
    manager_->setActiveProtocol(protocol_);

    // 发布状态
    status_pub_ = this->create_publisher<robot_interfaces::msg::MassageHeadState>("/massage_head_state", 10);

    //发布事件
    event_pub_ = this->create_publisher<robot_interfaces::msg::MassageHeadAttachEvent>("/massage_head_attach_event", 10);
    
    // 发布错误事件
    error_pub_ = this->create_publisher<robot_interfaces::msg::MassageHeadErrorEvent>("/massage_head_error_event", 10);
    
    // 发布串口重连恢复事件
    recovery_pub_ = this->create_publisher<robot_interfaces::msg::MassageHeadRecoveryEvent>("/massage_head_recovery_event", 10);

    // 初始化串口设备
    initializeSerial();

    // 初始化命令处理器（在串口初始化之后）
    initCommandProcessor();

    //创建总控制服务
    massage_head_switch_ = this->create_service<MassageHeadSwitchSrv>(
        "massage_head_switch",std::bind(&MassageHeadManageNode::handle_switch_service,this,std::placeholders::_1,
        std::placeholders::_2)); 

    
    // 创建JSON控制服务
    control_service_ = this->create_service<MassageHeadControlsrv>(
        "massage_head_control", std::bind(&MassageHeadManageNode::handle_json_service, this, std::placeholders::_1,
                std::placeholders::_2));

    //arm_set_tcp客户端发起请求
    arm_set_tcp_client_ = this->create_client<ArmSetTCPsrv>("/arm/set_tcp"); 

    // 状态发布定时器: 每500ms发布一次状态
    timer_ = this->create_wall_timer(500ms, [this]() 
    {
        robot_interfaces::msg::MassageHeadState st;
        st.header.stamp = this->now();
        st.header.frame_id = "massage_head_manager";
        
        // 检查设备是否在线
        if (!current_serial_number_.empty() && current_handle_id_ > 0) {
            // 设备在线：发布实时状态
            st.serial_number = utils::hexSpacesToHyphens(current_serial_number_);
            
            // 构建JSON状态
            nlohmann::json json_status;
            json_status["motor"] = current_motor_state_;
            json_status["led"] = current_led_state_;
            json_status["temperature"] = current_temp_actual_;
            
            st.massage_head_data = json_status.dump();
        } else {
            // 设备离线：发布空对象
            st.serial_number = "";
            st.massage_head_data = "{}";
        }
        
        // 发布状态
        status_pub_->publish(st);
    });

    // 识别定时器：200ms执行一次
    identify_timer_ = this->create_wall_timer(200ms, [this]() {
        if (protocol_) {
            // 1. 同步硬件状态到本地变量
            syncHardwareStatusToLocal();
            
            // 2. 检查设备是否离线
            if (protocol_->shouldClearDevice()) {
                if (current_handle_id_ > 0) {
                    RCLCPP_WARN(this->get_logger(), 
                               "设备离线");
                    
                    // 保存离线前的序列号用于事件通知
                    std::string offline_serial = current_serial_number_;
                    
                    // 清除所有状态
                    current_handle_id_ = 0;
                    current_handle_name_ = "Unknown";
                    current_serial_number_ = "";
                    current_motor_state_ = 0;
                    current_led_state_ = false;
                    current_temp_set_ = 0.0f;
                    current_temp_actual_ = 0.0f;
                    current_usage_time_ = 0;
                    
                    // 清除TCP缓存
                    tcp_cache_.is_valid = false;
                    tcp_cache_.last_handle_id = 0;
                    tcp_cache_.tcp_positions.clear();
                    last_logged_handle_id_ = 0;
                    
                    // 发布设备卸下事件
                    auto detach_event = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                    detach_event->header.stamp = this->now();
                    detach_event->header.frame_id = "massage_head_manager";
                    detach_event->serial_number = utils::hexSpacesToHyphens(offline_serial);
                    detach_event->is_attached = 0;  // 设备已卸下
                    detach_event->is_matched = false;
                    detach_event->event_description = "设备已卸下";
                    event_pub_->publish(*detach_event);
                    
                    RCLCPP_INFO(this->get_logger(), "已发布设备卸下事件: serial_number=%s", 
                               detach_event->serial_number.c_str());
                }
            }
            
            // 3. 使用命令队列发送识别命令（最低优先级）
            auto cmd_bytes = protocol_->createIdentifyCommand(0x01);
            enqueueIdentifyCommand(cmd_bytes);
            
            // 4. 设备在线时发送温度读取命令（状态查询优先级）
            if (current_handle_id_ > 0 && !current_serial_number_.empty()) {
                // 清空接收缓冲区，避免读取旧的温度数据
                {
                    std::lock_guard<std::mutex> lock(receive_buffer_mutex_);
                    if (!receive_buffer_.empty()) {
                        RCLCPP_DEBUG(this->get_logger(), "识别定时器: 清空接收缓冲区 %zu 字节旧数据", receive_buffer_.size());
                        receive_buffer_.clear();
                    }
                }
                
                auto temp_cmd = protocol_->createTemperatureReadCommand();
                if (!temp_cmd.empty()) {
                    enqueueStatusCommand(temp_cmd, "温度查询");
                }
            }
            
            // 5. 增加未响应计数
            protocol_->incrementNoResponseCount();
        }
    });

    // 累计用时读取定时器
    usage_time_read_timer_ = this->create_wall_timer(3600000ms, [this]()
    {
        if (protocol_)
        {
            auto cmd = protocol_->createUsageTimeReadCommand();
            enqueueStatusCommand(cmd, "累计用时查询");
        }
    });

    // 初始化挂起等待机制
    pending_processor_running_ = true;
    pending_requests_processor_ = std::thread(&MassageHeadManageNode::pendingRequestsProcessorLoop, this);

    // 请求挂起检查定时器
    pending_timer_ = this->create_wall_timer(5000ms, [this]() {
        pendingRequestsTimerCallback();
    });
}

MassageHeadManageNode::~MassageHeadManageNode()
{
    RCLCPP_INFO(this->get_logger(), "按摩头管理节点正在关闭...");

    // 0. 设置节点关闭标志，防止异步回调访问已销毁的资源
    node_shutting_down_.store(true);

    // 1. 首先停止所有定时器，防止新命令入队
    if (timer_) timer_->cancel();
    if (identify_timer_) identify_timer_->cancel();
    if (usage_time_read_timer_) usage_time_read_timer_->cancel();
    if (pending_timer_) pending_timer_->cancel();

    // 2. 停止挂起等待机制
    pending_processor_running_.store(false);
    
    if (pending_requests_processor_.joinable()) {
        try {
            pending_requests_processor_.join();
        } catch (...) {
            // 忽略异常
        }
    }

    // 3. 停止命令处理器
    stopCommandProcessor();

    // 4. 最后关闭串口
    closeSerial();
    
    // 5. 等待一小段时间让异步回调完成
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "按摩头管理节点已关闭");
}

void MassageHeadManageNode::closeSerial()
{
    if (serial_running_)
    {
        RCLCPP_INFO(this->get_logger(), "正在关闭串口...");

        // 设置退出标志，让循环条件立即失效
        serial_running_ = false;

        // 关闭串口
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open())
        {
            try
            {
                serial_driver_->port()->close();
                RCLCPP_INFO(this->get_logger(), "串口端口已关闭");
            }
            catch (const std::exception& ex)
            {
                RCLCPP_ERROR(this->get_logger(), "关闭串口失败: %s", ex.what());
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (serial_read_thread_.joinable())
        {
            RCLCPP_INFO(this->get_logger(), "等待读取线程结束...");

            for (int i = 0; i < 10 && serial_read_thread_.joinable(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (serial_read_thread_.joinable()) {
                serial_read_thread_.detach();
            }
        }

        serial_driver_.reset();
        io_context_.reset();
        receive_buffer_.clear();

        RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
}

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
        event_msg->header.frame_id = "massage_head_manager";
        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial);
        event_msg->is_attached = 1;  // 设备已安装，但序列号不匹配
        event_msg->is_matched = false;
        event_pub_->publish(*event_msg);

        return false;
    }

    RCLCPP_INFO(this->get_logger(), "序列号匹配验证成功: %s", current_serial.c_str());
    return true;
}


void MassageHeadManageNode::loadDatabaseCache()
{
    if (!database_manager_) {
        RCLCPP_WARN(this->get_logger(), "数据库管理器未初始化，无法加载缓存");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "开始加载数据库缓存...");

    try {
        // 获取数据库中所有按摩头配置
        auto all_heads = database_manager_->getAllMassageHead();
        RCLCPP_INFO(this->get_logger(), "数据库中共有 %zu 个按摩头配置", all_heads.size());

        // 建立serial_number到按摩头信息的映射缓存
        for (const auto& head : all_heads) {
            if (head.serial_number.empty()) {
                RCLCPP_WARN(this->get_logger(), "跳过无效配置: ID=%s, serial_number为空", head.id.c_str());
                continue;
            }

            DatabaseCache::MassageHeadInfo info;
            info.name = head.name;
            info.serial_number = head.serial_number;
            info.config_data = head.config_data;  // 保存原始config_data

            // 从config_data解析TCP坐标
            info.tcp_positions = DatabaseManager::parseTcpFromConfigData(head.config_data);

            // 以serial_number为key存入缓存
            db_cache_.serial_to_info[head.serial_number] = info;

            RCLCPP_INFO(this->get_logger(),
                       "✓ 缓存按摩头配置: serial_number=%s, 名称=%s",
                       head.serial_number.c_str(), info.name.c_str());
        }

        RCLCPP_INFO(this->get_logger(),
                   "数据库缓存加载完成，共缓存 %zu 个按摩头配置",
                   db_cache_.serial_to_info.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "加载数据库缓存异常: %s", e.what());
    }

    db_cache_.is_loaded = true;
}

void MassageHeadManageNode::handle_json_control_service(const std::shared_ptr<MassageHeadControlsrv::Request> request,
    const std::shared_ptr<MassageHeadControlsrv::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到JSON控制请求");

    try {
        // 从单独的字段获取 serial_number，并转换格式："5A-A5-06..." -> "5A A5 06..."
        std::string serial_number = utils::hexHyphensToSpaces(request->serial_number);
        
        // 解析JSON数据 - 支持两种格式
        nlohmann::json params;
        nlohmann::json json_data = nlohmann::json::parse(request->command_params);
        
        // 检查是否是嵌套格式：{"properties": [{...}]}
        if (json_data.contains("properties") && json_data["properties"].is_array() && 
            !json_data["properties"].empty()) {
            // 嵌套格式：提取第一个配置项的参数
            auto first_item = json_data["properties"][0];
            
            // 直接读取扁平化的字段
            params = first_item;
        } else {
            // 扁平化格式：直接使用
            params = json_data;
        }

        RCLCPP_INFO(this->get_logger(), "处理序列号: %s , 参数: %s", 
                   serial_number.c_str(),  params.dump().c_str());

        bool overall_success = true;

        // 1. 映射匹配检查
        if (!validateSerialNumberMapping(serial_number)) {
            // 序列号不匹配，添加到挂起队列而不是立即失败
            std::string request_id = addPendingRequest(serial_number, params, nullptr, true, params.dump());
            RCLCPP_WARN(this->get_logger(), "序列号不匹配: %s，请求已挂起等待正确按摩头安装 (request_id=%s)",
                       serial_number.c_str(), request_id.c_str());

            // 发布不匹配事件
            auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
            event_msg->header.stamp = this->now();
            event_msg->header.frame_id = "massage_head_manager";
            event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
            event_msg->is_attached = 0;  // 未安装正确的按摩头
            event_msg->is_matched = false;  // 不匹配
            event_msg->event_description = "序列号不匹配，请求已挂起: " + serial_number;
            event_pub_->publish(*event_msg);

            response->success = false;
            response->result = "序列号不匹配，请求已挂起等待正确按摩头安装 (request_id: " + request_id + ")";
            return;
        }

        // ==================== RET能量控制 (ret: 10-100档) [仅V2协议] ====================
        if (params.contains("ret")) {
            int ret_value = params["ret"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (!protocol_v2) {
                RCLCPP_DEBUG(this->get_logger(), "RET参数被忽略（当前使用V1协议）");
            } else if (ret_value >= 10 && ret_value <= 100) {
                auto cmd = protocol_v2->createPresetCommand(PresetCommand::RET_ENERGY, static_cast<uint8_t>(ret_value));
                if (!cmd.empty()) {
                    enqueueControlCommand(cmd, "RET能量预设:" + std::to_string(ret_value) + "档");
                    RCLCPP_INFO(this->get_logger(), "RET能量预设: %d档", ret_value);
                } else {
                    overall_success = false;
                }
            } else if (ret_value == 0) {
                auto cmd = protocol_v2->createRETCommand(RETCommand::STOP);
                enqueueControlCommand(cmd, "停止RET");
                RCLCPP_INFO(this->get_logger(), "停止RET");
            } else {
                RCLCPP_ERROR(this->get_logger(), "RET能量超出范围: %d (应在10-100或0关闭)", ret_value);
                overall_success = false;
            }
        }

        // ==================== 微电能量控制 (micro_electric: 1-100档) [仅V2协议] ====================
        if (params.contains("micro_electric")) {
            int micro_value = params["micro_electric"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (!protocol_v2) {
                RCLCPP_DEBUG(this->get_logger(), "微电参数被忽略（当前使用V1协议）");
            } else if (micro_value >= 1 && micro_value <= 100) {
                auto cmd = protocol_v2->createPresetCommand(PresetCommand::MICRO_ELECTRIC, static_cast<uint8_t>(micro_value));
                if (!cmd.empty()) {
                    enqueueControlCommand(cmd, "微电能量预设:" + std::to_string(micro_value) + "档");
                    RCLCPP_INFO(this->get_logger(), "微电能量预设: %d档", micro_value);
                } else {
                    overall_success = false;
                }
            } else if (micro_value == 0) {
                auto cmd = protocol_v2->createMicroElectricCommand(MicroElectricCommand::TOGGLE);
                enqueueControlCommand(cmd, "微电启停");
                RCLCPP_INFO(this->get_logger(), "微电停止");
            } else {
                RCLCPP_ERROR(this->get_logger(), "微电能量超出范围: %d (应在1-100或0关闭)", micro_value);
                overall_success = false;
            }
        }

        // ==================== 负压吸力控制 (negative_pressure: 1-16档) [仅V2协议] ====================
        if (params.contains("negative_pressure")) {
            int negative_value = params["negative_pressure"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (!protocol_v2) {
                RCLCPP_DEBUG(this->get_logger(), "负压参数被忽略（当前使用V1协议）");
            } else if (negative_value >= 1 && negative_value <= 16) {
                auto cmd = protocol_v2->createPresetCommand(PresetCommand::NEGATIVE_SUCTION, static_cast<uint8_t>(negative_value));
                if (!cmd.empty()) {
                    enqueueControlCommand(cmd, "负压吸力预设:" + std::to_string(negative_value) + "档");
                    RCLCPP_INFO(this->get_logger(), "负压吸力预设: %d档", negative_value);
                } else {
                    overall_success = false;
                }
            } else if (negative_value == 0) {
                auto cmd = protocol_v2->createNegativePressureCommand(NegativePressureCommand::TOGGLE);
                enqueueControlCommand(cmd, "负压启停");
                RCLCPP_INFO(this->get_logger(), "负压停止");
            } else {
                RCLCPP_ERROR(this->get_logger(), "负压吸力超出范围: %d (应在1-16或0关闭)", negative_value);
                overall_success = false;
            }
        }

        // ==================== 发热片温度控制 (temperature: 10-75°C for V2, 30-60°C for V1) ====================
        if (params.contains("temperature")) {
            int temp_value = params["temperature"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (protocol_v2) {
                // V2 协议：使用发热片温度预设命令，范围 10-75°C
                if (temp_value >= 10 && temp_value <= 75) {
                    auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, static_cast<uint8_t>(temp_value));
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "发热片温度预设:" + std::to_string(temp_value) + "°C");
                        RCLCPP_INFO(this->get_logger(), "发热片温度预设(V2): %d°C", temp_value);
                    } else {
                        overall_success = false;
                    }
                } else if (temp_value == 0) {
                    RCLCPP_INFO(this->get_logger(), "关闭发热片（温度=0）");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "发热片温度超出范围(V2): %d (应在10-75°C或0关闭)", temp_value);
                    overall_success = false;
                }
            } else {
                // V1 协议：使用原有的 handleTempControlJson，范围 30-60°C
                float temp_float = static_cast<float>(temp_value);
                bool temp_success = handleTempControlJson(temp_float);
                if (!temp_success) {
                    RCLCPP_WARN(this->get_logger(), "温度控制失败(V1)");
                    overall_success = false;
                }
            }
        }

        // ==================== 电机控制 (motor: 0-停止, 1-正转, 2-反转) ====================
        if (params.contains("motor")) {
            int motor_state = params["motor"].get<int>();
            
            if (motor_state >= 0 && motor_state <= 2) {
                bool motor_success = handleMotorControlJson(motor_state);
                if (!motor_success) {
                    RCLCPP_WARN(this->get_logger(), "电机控制失败");
                    overall_success = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "电机状态无效: %d (应为0/1/2)", motor_state);
                overall_success = false;
            }
        }

        // ==================== 冲击波控制 (shock_wave: 脉冲数) [仅V2协议] ====================
        if (params.contains("shock_wave")) {
            int shock_value = params["shock_wave"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (!protocol_v2) {
                RCLCPP_DEBUG(this->get_logger(), "冲击波参数被忽略（当前使用V1协议）");
            } else if (shock_value > 0) {
                auto cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
                enqueueControlCommand(cmd, "冲击波启动");
                RCLCPP_INFO(this->get_logger(), "冲击波启动");
            } else {
                auto cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
                enqueueControlCommand(cmd, "冲击波停止");
                RCLCPP_INFO(this->get_logger(), "冲击波停止");
            }
        }

        // ==================== 冲击波频率控制 (frequency: 1-21Hz) [仅V2协议] ====================
        if (params.contains("frequency")) {
            int freq_value = params["frequency"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (!protocol_v2) {
                RCLCPP_DEBUG(this->get_logger(), "频率参数被忽略（当前使用V1协议）");
            } else if (freq_value >= 1 && freq_value <= 21) {
                auto cmd = protocol_v2->createPresetCommand(PresetCommand::SHOCK_WAVE_FREQ, static_cast<uint8_t>(freq_value));
                if (!cmd.empty()) {
                    enqueueControlCommand(cmd, "冲击波频率预设:" + std::to_string(freq_value) + "Hz");
                    RCLCPP_INFO(this->get_logger(), "冲击波频率预设: %dHz", freq_value);
                } else {
                    overall_success = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "冲击波频率超出范围: %d (应在1-21Hz)", freq_value);
                overall_success = false;
            }
        }

        // ==================== 冲击波能量控制 (energy: 1-16档) [仅V2协议] ====================
        if (params.contains("energy")) {
            int energy_value = params["energy"].get<int>();
            auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
            
            if (!protocol_v2) {
                RCLCPP_DEBUG(this->get_logger(), "能量参数被忽略（当前使用V1协议）");
            } else if (energy_value >= 1 && energy_value <= 16) {
                auto cmd = protocol_v2->createPresetCommand(PresetCommand::SHOCK_WAVE_ENERGY, static_cast<uint8_t>(energy_value));
                if (!cmd.empty()) {
                    enqueueControlCommand(cmd, "冲击波能量预设:" + std::to_string(energy_value) + "档");
                    RCLCPP_INFO(this->get_logger(), "冲击波能量预设: %d档", energy_value);
                } else {
                    overall_success = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "冲击波能量超出范围: %d (应在1-16档)", energy_value);
                overall_success = false;
            }
        }

        response->success = overall_success;
        response->result = overall_success ? "所有命令执行成功" : "部分或全部命令执行失败";

    } catch (const nlohmann::json::exception& e) {
        response->success = false;
        response->result = "JSON 解析错误: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "JSON 解析错误: %s", e.what());
    } catch (const std::exception& e) {
        response->success = false;
        response->result = "处理错误: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "处理错误: %s", e.what());
    }
}

/**
 * @brief 从JSON配置执行控制命令
 * @details 支持扁平式JSON参数控制各种功能，用于数据库配置驱动的开关控制
 */
bool MassageHeadManageNode::executeSwitchFromDatabase(const nlohmann::json& params, std::string& message)
{
    bool overall_success = true;
    std::vector<std::string> success_msgs;
    std::vector<std::string> error_msgs;
    
    try {
        auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
        
        // ==================== RET控制 ====================
        if (params.contains("ret_enabled")) {
            bool ret_enabled = params["ret_enabled"].get<bool>();
            if (protocol_v2) {
                if (ret_enabled) {
                    // 启动RET
                    uint8_t energy = params.value("ret_energy", 10);
                    auto cmd = protocol_v2->createPresetCommand(PresetCommand::RET_ENERGY, energy);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "RET能量预设:" + std::to_string(energy) + "档");
                        success_msgs.push_back("RET启动(" + std::to_string(energy) + "档)");
                    } else {
                        error_msgs.push_back("RET启动失败");
                        overall_success = false;
                    }
                } else {
                    // 停止RET
                    auto cmd = protocol_v2->createRETCommand(RETCommand::STOP);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "停止RET");
                        success_msgs.push_back("RET停止");
                    } else {
                        error_msgs.push_back("RET停止失败");
                        overall_success = false;
                    }
                }
            }
        }
        
        // ==================== 微电控制 ====================
        if (params.contains("micro_enabled")) {
            bool micro_enabled = params["micro_enabled"].get<bool>();
            if (protocol_v2) {
                if (micro_enabled) {
                    // 启动微电
                    uint8_t energy = params.value("micro_energy", 25);
                    auto cmd = protocol_v2->createPresetCommand(PresetCommand::MICRO_ELECTRIC, energy);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "微电能量预设:" + std::to_string(energy) + "档");
                        success_msgs.push_back("微电启动(" + std::to_string(energy) + "档)");
                    } else {
                        error_msgs.push_back("微电启动失败");
                        overall_success = false;
                    }
                } else {
                    // 停止微电
                    auto cmd = protocol_v2->createMicroElectricCommand(MicroElectricCommand::TOGGLE);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "微电启停");
                        success_msgs.push_back("微电停止");
                    } else {
                        error_msgs.push_back("微电停止失败");
                        overall_success = false;
                    }
                }
            }
        }
        
        // ==================== 负压控制 ====================
        if (params.contains("negative_pressure_enabled")) {
            bool negative_enabled = params["negative_pressure_enabled"].get<bool>();
            if (protocol_v2) {
                if (negative_enabled) {
                    // 启动负压
                    uint8_t suction = params.value("negative_suction", 5);
                    auto cmd = protocol_v2->createPresetCommand(PresetCommand::NEGATIVE_SUCTION, suction);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "负压吸力预设:" + std::to_string(suction) + "档");
                        success_msgs.push_back("负压启动(" + std::to_string(suction) + "档)");
                    } else {
                        error_msgs.push_back("负压启动失败");
                        overall_success = false;
                    }
                } else {
                    // 停止负压
                    auto cmd = protocol_v2->createNegativePressureCommand(NegativePressureCommand::TOGGLE);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "负压启停");
                        success_msgs.push_back("负压停止");
                    } else {
                        error_msgs.push_back("负压停止失败");
                        overall_success = false;
                    }
                }
            }
        }
        
        // ==================== 温度控制 ====================
        if (params.contains("temperature")) {
            int temp_value = params["temperature"].get<int>();
            if (protocol_v2) {
                // V2协议：10-75°C
                if (temp_value >= 10 && temp_value <= 75) {
                    auto cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, static_cast<uint8_t>(temp_value));
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "发热片温度预设:" + std::to_string(temp_value) + "°C");
                        success_msgs.push_back("温度设置(" + std::to_string(temp_value) + "°C)");
                    } else {
                        error_msgs.push_back("温度设置失败");
                        overall_success = false;
                    }
                } else if (temp_value == 0) {
                    success_msgs.push_back("温度关闭");
                }
            } else {
                // V1协议：30-60°C
                if (handleTempControlJson(static_cast<float>(temp_value))) {
                    success_msgs.push_back("温度设置(V1:" + std::to_string(temp_value) + "°C)");
                } else {
                    error_msgs.push_back("温度设置失败(V1)");
                    overall_success = false;
                }
            }
        }
        
        // ==================== 电机控制 ====================
        if (params.contains("motor_state")) {
            int motor_state = params["motor_state"].get<int>();
            if (motor_state >= 0 && motor_state <= 2) {
                if (handleMotorControlJson(motor_state)) {
                    std::string motor_msg = (motor_state == 0) ? "停止" : 
                                           (motor_state == 1) ? "正转" : "反转";
                    success_msgs.push_back("电机" + motor_msg);
                } else {
                    error_msgs.push_back("电机控制失败");
                    overall_success = false;
                }
            }
        }
        
        // ==================== 冲击波控制 ====================
        if (params.contains("shock_wave_enabled")) {
            bool shock_enabled = params["shock_wave_enabled"].get<bool>();
            if (protocol_v2) {
                if (shock_enabled) {
                    // 启动冲击波
                    uint8_t energy = params.value("shock_wave_energy", 1);
                    uint8_t freq = params.value("shock_wave_freq", 1);
                    
                    // 设置能量
                    auto energy_cmd = protocol_v2->createPresetCommand(PresetCommand::SHOCK_WAVE_ENERGY, energy);
                    if (!energy_cmd.empty()) {
                        enqueueControlCommand(energy_cmd, "冲击波能量预设:" + std::to_string(energy) + "档");
                    }
                    
                    // 设置频率
                    auto freq_cmd = protocol_v2->createPresetCommand(PresetCommand::SHOCK_WAVE_FREQ, freq);
                    if (!freq_cmd.empty()) {
                        enqueueControlCommand(freq_cmd, "冲击波频率预设:" + std::to_string(freq) + "Hz");
                    }
                    
                    // 启动冲击波
                    auto toggle_cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
                    if (!toggle_cmd.empty()) {
                        enqueueControlCommand(toggle_cmd, "冲击波启动");
                        success_msgs.push_back("冲击波启动(" + std::to_string(energy) + "档," + std::to_string(freq) + "Hz)");
                    } else {
                        error_msgs.push_back("冲击波启动失败");
                        overall_success = false;
                    }
                } else {
                    // 停止冲击波
                    auto cmd = protocol_v2->createShockWaveCommand(ShockWaveCommand::TOGGLE);
                    if (!cmd.empty()) {
                        enqueueControlCommand(cmd, "冲击波停止");
                        success_msgs.push_back("冲击波停止");
                    } else {
                        error_msgs.push_back("冲击波停止失败");
                        overall_success = false;
                    }
                }
            }
        }
        
        // 构建返回消息
        if (overall_success) {
            message = "成功: " + (success_msgs.empty() ? "无操作" : 
                     std::accumulate(success_msgs.begin(), success_msgs.end(), std::string(),
                                    [](const std::string& a, const std::string& b) {
                                        return a.empty() ? b : a + ", " + b;
                                    }));
        } else {
            message = "失败: " + (error_msgs.empty() ? "未知错误" :
                     std::accumulate(error_msgs.begin(), error_msgs.end(), std::string(),
                                    [](const std::string& a, const std::string& b) {
                                        return a.empty() ? b : a + ", " + b;
                                    }));
        }
        
        RCLCPP_INFO(this->get_logger(), "executeSwitchFromDatabase: %s", message.c_str());
        
    } catch (const std::exception& e) {
        message = "异常: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "executeSwitchFromDatabase异常: %s", e.what());
        overall_success = false;
    }
    
    return overall_success;
}

void MassageHeadManageNode::handle_switch_service(const std::shared_ptr<MassageHeadSwitchSrv::Request> request,
    const std::shared_ptr<MassageHeadSwitchSrv::Response> response)
{
    // 转换 serial_number 格式："5A-A5-06..." -> "5A A5 06..."
    std::string serial_number = utils::hexHyphensToSpaces(request->serial_number);
    
    // 验证序列号匹配
    if (!request->serial_number.empty() && !validateSerialNumberMapping(serial_number)) {
        response->success = false;
        response->message = "序列号不匹配: 请求=" + request->serial_number + ", 实际=" + utils::hexSpacesToHyphens(current_serial_number_);
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // 将字符串命令转换为整数
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
    
    // 定义命令常量
    constexpr uint8_t CMD_START = 1;
    
    bool success = false;
    std::string message = "";
    
    try
    {
        // 从数据库获取switch_command配置
        std::string switch_command_json = database_manager_->getSwitchCommandBySerialNumber(serial_number);
        
        if (switch_command_json.empty()) {
            response->success = false;
            response->message = "未找到该按摩头的开关配置: " + serial_number;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // 解析JSON配置
        nlohmann::json config_json;
        try {
            config_json = nlohmann::json::parse(switch_command_json);
        } catch (const nlohmann::json::exception& e) {
            response->success = false;
            response->message = "解析switch_command JSON失败: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // 根据命令类型选择start或stop配置
        std::string action_key = (command == CMD_START) ? "start" : "stop";
        
        if (!config_json.contains(action_key)) {
            response->success = false;
            response->message = "switch_command中缺少" + action_key + "配置";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        nlohmann::json action_params = config_json[action_key];
        
        RCLCPP_INFO(this->get_logger(), "执行%s命令，配置: %s", 
                   action_key.c_str(), action_params.dump().c_str());
        
        // 执行控制命令（调用handle_json_control_service的实现逻辑）
        success = executeSwitchFromDatabase(action_params, message);
    }
    catch(const std::exception& e)
    {
        success = false;
        message = "处理控制命令时发生异常: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }

    //响应服务
    response->success = success;
    response->message = message;
    RCLCPP_INFO(this->get_logger(),"控制服务响应:success = %s, message = %s",
               success ? "true" : "false", message.c_str());
}


/**
 * 
 * @brief
 * 处理按摩头控制服务
 * 
 */
void MassageHeadManageNode::handle_json_service(const std::shared_ptr<MassageHeadControlsrv::Request> request,
    const std::shared_ptr<MassageHeadControlsrv::Response> response)
    {
        // 处理按摩头控制请求 - 直接转发给 JSON 处理函数
        // 注意：serial_number 格式转换将在 handle_json_control_service 中完成
        try {
            RCLCPP_INFO(this->get_logger(), "收到按摩头控制请求: serial_number=%s , params=%s",
                       request->serial_number.c_str(), request->command_params.c_str());

            
            auto json_request = std::make_shared<MassageHeadControlsrv::Request>();
            json_request->serial_number = request->serial_number;
            json_request->command_params = request->command_params;
            
            auto json_response = std::make_shared<MassageHeadControlsrv::Response>();
            handle_json_control_service(json_request, json_response);
            
            // 返回结果
            response->success = json_response->success;
            response->result = json_response->result;
            
            RCLCPP_INFO(this->get_logger(), "按摩头控制服务响应: success=%s, result=%s",
                       response->success ? "true" : "false", response->result.c_str());

        } catch (const nlohmann::json::exception& e) {
            response->success = false;
            response->result = "JSON解析错误: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "JSON解析错误: %s", e.what());
        } catch (const std::exception& e) {
            response->success = false;
            response->result = "处理错误: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "处理错误: %s", e.what());
        }
    }


/**========================================================================= */

//发送电机控制命令
bool MassageHeadManageNode::sendMotorCommand(uint8_t state)
{
    try
    {
        // 电机状态参数说明：
        // 0 = 停止
        // 1 = 正转（顺时针）
        // 2 = 反转（逆时针）
        auto cmd = protocol_->createCommandPacket(CommandType::SET_MOTOR_STATE, static_cast<float>(state));
        if(!cmd.empty())
        {
            const char* state_str = (state == 0) ? "停止" : (state == 1) ? "正转" : (state == 2) ? "反转" : "未知";
            std::string desc = std::string("电机控制:") + state_str;
            
            // 使用命令队列发送高优先级控制命令
            bool queued = enqueueControlCommand(cmd, desc);
            if (queued) {
                RCLCPP_INFO(this->get_logger(), "电机命令已入队：%s (%d)", state_str, state);
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "电机命令入队失败：%s (%d)", state_str, state);
                return false;
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "发送命令异常：%s", e.what());
    }
    return false;
}

//发送温度开关控制命令
bool MassageHeadManageNode::sendTempCommand(float temperature)
{
    try
    {
        std::string desc;
        if (temperature > 0.0f)
        {
            desc = "加热启用:" + std::to_string(static_cast<int>(temperature)) + "°C";
            RCLCPP_INFO(this->get_logger(), "发送加热启用命令，目标温度:%.1f°C", temperature);
        }
        else
        {
            desc = "关闭加热";
            RCLCPP_INFO(this->get_logger(), "关闭加热");
        }
        
        // 使用温度设置接口：temperature为0表示关闭加热，>0表示启用并设置温度
        auto cmd = protocol_->createTemperatureSetCommand(temperature);
        if(!cmd.empty())
        {
            // 使用命令队列发送高优先级控制命令
            bool queued = enqueueControlCommand(cmd, desc);
            if (queued) {
                RCLCPP_INFO(this->get_logger(), "温度命令已入队");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "温度命令入队失败");
                return false;
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "发送命令异常：%s", e.what());
    }
    return false;
}

//发送LED开关控制命令
bool MassageHeadManageNode::sendLedCommand(bool on)
{
    try
    {
        // LED状态参数说明：
        // false (0.0f) = LED关闭
        // true  (1.0f) = LED开启
        auto cmd = protocol_->createCommandPacket(CommandType::SET_LED_STATE, on ? 1.0f : 0.0f);
        if(!cmd.empty())
        {
            std::string desc = std::string("LED控制:") + (on ? "开启" : "关闭");
            
            // 使用命令队列发送高优先级控制命令
            bool queued = enqueueControlCommand(cmd, desc);
            if (queued) {
                RCLCPP_INFO(this->get_logger(), "LED命令已入队：%s", on ? "开启" : "关闭");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "LED命令入队失败");
                return false;
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "发送命令异常：%s", e.what());
    }
    return false;
}

/*=====同步硬件反馈=======*/
void MassageHeadManageNode::syncHardwareStatusToLocal()
{
    if (protocol_)
    {
        // 使用 manager_->getCurrentStatus() 保证对协议状态的互斥访问
        auto hardware_status = manager_->getCurrentStatus();

        // 同步实际温度和状态
        current_motor_state_ = hardware_status.motor_state;
        current_led_state_ = hardware_status.led_state;
        current_temp_set_ = hardware_status.temp_set;
        current_temp_actual_ = hardware_status.temp_actual;
        current_usage_time_ = hardware_status.usage_time;
    }
}
/*================================*/

void MassageHeadManageNode::onSerialData(const std::string &hex_str)
{
    // 打印收到的原始串口十六进制数据
    RCLCPP_DEBUG(this->get_logger(), "收到串口数据: '%s'", hex_str.c_str());

    auto bytes = utils::hexStringToBytes(hex_str);
    if (bytes.empty()) {
        RCLCPP_WARN(this->get_logger(), "收到空串口数据: '%s'", hex_str.c_str());
        return;
    }

    // 将 bytes 交给 manager 解析并获取解析结果
    auto parse_res = manager_->parseIncomingData(bytes);


    // 手柄识别数据包时更新serial_number
    bool is_new_device_online = false;
    if (parse_res.type == BaseParseResult::MessageType::HANDLE_INFO && parse_res.success) {
        // 收到识别包响应，重置未响应计数器和等待标志
        if (protocol_) {
            protocol_->resetNoResponseCount();
            static int reset_count = 0;
            if (reset_count++ % 10 == 0) {
                RCLCPP_DEBUG(this->get_logger(), "收到识别包，重置计数器");
            }
        }
        
        // 检查是否是新设备上线
        if (parse_res.handle_id > 0) {
            bool is_truly_new = false;
            
            if (last_logged_handle_id_ == 0) {
                // 从离线状态恢复
                if (!current_serial_number_.empty()) {
                    // 比较序列号判断是否是同一设备
                    std::vector<uint8_t> new_packet_bytes = utils::hexStringToBytes(hex_str);
                    if (new_packet_bytes.size() >= 9) {
                        std::vector<uint8_t> new_id_packet(new_packet_bytes.begin(), new_packet_bytes.begin() + 9);
                        std::string new_serial = utils::bytesToDatabaseHexString(new_id_packet);
                        is_truly_new = (new_serial != current_serial_number_);
                    } else {
                        is_truly_new = true;  
                    }
                } else {
                    is_truly_new = true;
                }
            } else if (parse_res.handle_id != last_logged_handle_id_) {
                is_truly_new = true;
            }
            
            if (is_truly_new) {
                is_new_device_online = true;
                RCLCPP_INFO(this->get_logger(), "检测到新设备上线: ID=%d ", 
                           parse_res.handle_id);
            } else if (last_logged_handle_id_ == 0) {
                // 同一设备从离线恢复
                RCLCPP_INFO(this->get_logger(), "设备从离线状态恢复: ID=%d", parse_res.handle_id);
            }
            
            last_logged_handle_id_ = parse_res.handle_id;
        }

        // 更新内部状态
        current_handle_id_ = static_cast<uint8_t>(parse_res.handle_id);
        current_handle_name_ = parse_res.handle_name;

        // 确保只存储9字节的识别包，避免粘包
        auto bytes = utils::hexStringToBytes(hex_str);
        if (bytes.size() >= 9) {
            std::vector<uint8_t> id_packet(bytes.begin(), bytes.begin() + 9);
            std::string new_serial = utils::bytesToDatabaseHexString(id_packet);
            
            // 只有当序列号改变时才打印日志
            if (new_serial != current_serial_number_) {
                RCLCPP_INFO(this->get_logger(), "更新序列号: %s, 设备ID: %d, 设备名: %s", 
                           new_serial.c_str(), current_handle_id_, current_handle_name_.c_str());
            }
            current_serial_number_ = new_serial;
        } else {
            current_serial_number_ = hex_str;  
            RCLCPP_WARN(this->get_logger(), "包长不足9字节，使用原始数据: %s (长度=%zu)", hex_str.c_str(), bytes.size());
        }
    } else if (parse_res.type != BaseParseResult::MessageType::HANDLE_INFO) 
    {
    }

    // 同步硬件反馈
    syncHardwareStatusToLocal();

    // 重置初始化状态
    if (is_new_device_online && parse_res.handle_id > 0) {
        RCLCPP_INFO(this->get_logger(), "新设备上线");
    }

    // 所有状态更新依赖设备的响应包
    if (is_new_device_online && parse_res.handle_id > 0) {
        RCLCPP_INFO(this->get_logger(), "读取设备温度");
        try {
            // 读取设备温度状态
            if (protocol_) {
                auto temp_cmd = protocol_->createTemperatureReadCommand();
                if (!temp_cmd.empty()) {
                    RCLCPP_INFO(this->get_logger(), "发送温度读取命令");
                    sendSerialData(temp_cmd);
                }
            }

            // 自动设置TCP坐标
            RCLCPP_INFO(this->get_logger(), "发送TCP设置请求");
            // 使用serial_number进行TCP设置
            if (!current_serial_number_.empty()) {
                sendTcpSetRequestBySerialNumber(current_serial_number_);
            } else {
                RCLCPP_WARN(this->get_logger(), "无法获取有效的serial_number，跳过TCP设置");
            }

            // 检查并处理挂起的请求
            processPendingRequests();

            RCLCPP_INFO(this->get_logger(), "设备初始状态读取完成");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "设备异常: %s", e.what());
        }
    }
}


// 获取本地状态
robot_interfaces::msg::MassageHeadState MassageHeadManageNode::getLocalStatus()
{
    robot_interfaces::msg::MassageHeadState status;
    status.header.stamp = this->now();

    // 直接返回当前的serial_number
    if (current_handle_id_ > 0) {
        status.serial_number = utils::hexSpacesToHyphens(current_serial_number_);
        
        // 更新TCP缓存
        if (!tcp_cache_.is_valid || tcp_cache_.last_handle_id != current_handle_id_) {
            updateTcpCache();
        }
    } else {
        status.serial_number = "";
    }

    return status;
}

// 更新TCP缓存
void MassageHeadManageNode::updateTcpCache()
{
    // 尝试从本地数据库缓存获取TCP数据
    if (db_cache_.is_loaded && !current_serial_number_.empty() &&
        db_cache_.serial_to_info.find(current_serial_number_) != db_cache_.serial_to_info.end()) {

        const auto& massage_head_info = db_cache_.serial_to_info[current_serial_number_];

        if (!massage_head_info.tcp_positions.empty()) {
            tcp_cache_.is_valid = true;
            tcp_cache_.last_handle_id = current_handle_id_;
            tcp_cache_.tcp_positions = massage_head_info.tcp_positions;
            RCLCPP_DEBUG(this->get_logger(),
                        "缓存按摩头 serial_number=%s 的TCP数据",
                        current_serial_number_.c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                        "本地缓存中按摩头 serial_number=%s 没有TCP数据",
                        current_serial_number_.c_str());
        }
    } else {
        // 首次遇到时警告
        static std::set<std::string> warned_serials;
        if (warned_serials.find(current_serial_number_) == warned_serials.end()) {
            RCLCPP_WARN(this->get_logger(),
                       "本地缓存中未找到按摩头 serial_number=%s",
                       current_serial_number_.c_str());
            warned_serials.insert(current_serial_number_);
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                        "本地缓存中未找到按摩头 serial_number=%s",
                        current_serial_number_.c_str());
        }
    }
}

// 生成JSON格式的按摩头状态
std::string MassageHeadManageNode::generateJsonStatus()
{
    nlohmann::json json_status;
    
    // 获取当前状态
    auto protocol_status = manager_->getCurrentStatus();
    
    json_status["motor"] = protocol_status.motor_state;
    json_status["led"] = protocol_status.led_state;
    json_status["temperature"] = protocol_status.temp_actual;

    return json_status.dump();
}

// 添加协议切换函数
void MassageHeadManageNode::switchProtocol(std::shared_ptr<IProtocol> new_protocol)
{
    protocol_ = std::dynamic_pointer_cast<ProtocolV1>(new_protocol);
    if (protocol_) 
    {
        manager_->setActiveProtocol(protocol_);
        RCLCPP_INFO(this->get_logger(), "协议切换完成");
    }
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "协议切换失败");
    }
}

//===================== 串口功能实现方法 =====================//
/**
 * @brief 初始化串口设备
 * 打开串口设备并启动读取线程
 * @return bool 初始化成功返回true，失败返回false
 */
bool MassageHeadManageNode::initializeSerial()
{
    // 使用互斥锁防止并发调用
    std::lock_guard<std::mutex> lock(serial_init_mutex_);
    
    // 检查是否已经在重连中
    if (is_reconnecting_.load()) {
        return false;
    }
    
    // 设置重连标志
    is_reconnecting_.store(true);
    
    RCLCPP_INFO(this->get_logger(), "初始化串口设备: %s", device_name_.c_str());

    // 如果串口已经打开，先关闭它
    if (serial_running_)
    {
        closeSerial();
    }
    // 等待串口设备重新准备就绪
    RCLCPP_INFO(this->get_logger(), "等待设备准备就绪...");
    std::this_thread::sleep_for(500ms);
    try {
        // 创建IO上下文和串口驱动
        io_context_ = std::make_shared<IoContext>();
        // 打开串口
        bool serial_opened = false;
        for (int attempt = 0; attempt < 5 && rclcpp::ok(); ++attempt)
        {
            try
            {
                serial_driver_ = std::make_shared<SerialDriver>(*io_context_);
                auto config = SerialPortConfig(baudrate_, FlowControl::NONE, Parity::NONE, StopBits::ONE);
                serial_driver_->init_port(device_name_, config);
                serial_driver_->port()->open();

                if (serial_driver_->port()->is_open())
                {
                    RCLCPP_INFO(this->get_logger(), "串口打开成功: %s", device_name_.c_str());
                    serial_opened = true;
                    break;
                }
            }
            catch (const std::exception& ex)
            {
                std::this_thread::sleep_for((attempt + 1) * 200ms);
            }
        }

        if (!serial_opened || !serial_driver_ || !serial_driver_->port()->is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "串口多次打开失败");
            auto error_code = std::make_shared<robot_interfaces::msg::MassageHeadErrorEvent>();
                error_code->header.stamp = this->now();
                error_code->header.frame_id = "massage_head_manager";
                error_code->event_type = 3;  //串口设备错误
                error_code->event_msg = "串口设备打开失败";
                error_pub_->publish(*error_code);
            is_reconnecting_.store(false);  // 重置重连标志
            return false;
        }
        // 启动读取线程
        serial_running_ = true;
        serial_read_thread_ = std::thread(&MassageHeadManageNode::serialReadLoop, this);

        RCLCPP_INFO(this->get_logger(), "串口设备初始化完成");
        is_reconnecting_.store(false);  // 重置重连标志
        return true;

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "串口初始化失败: %s", ex.what());
        is_reconnecting_.store(false);  // 重置重连标志
        return false;
    }
}

/**
 * @brief 串口读取循环线程函数
 * @details 在单独线程中持续读取串口数据：
 *          1. 检查串口状态
 *          2. 读取数据到缓冲区
 *          3. 交给协议层处理
 *          4. 异常时尝试重连
 *          5. 重连失败后等待5s再试
 */
void MassageHeadManageNode::serialReadLoop()
{
    RCLCPP_INFO(this->get_logger(), "串口读取线程启动");
    std::vector<uint8_t> buffer(256);
    int reconnect_attempts = 0;
    bool error_event_published = false;  // 用于控制错误事件的发布频率

    while (serial_running_ && rclcpp::ok()) {
        try {
            // 快速检查退出标志
            if (!serial_running_.load()) {
                RCLCPP_INFO(this->get_logger(), "检测到退出信号，串口读取线程准备退出");
                break;
            }

            if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
                // 如果是从断开状态恢复，重置相关标志
                if (reconnect_attempts > 0) {
                    reconnect_attempts = 0;  // 重置重连计数
                }
                error_event_published = false;  // 重置错误事件标志

                try {
                    size_t bytes_read = serial_driver_->port()->receive(buffer);

                    if (bytes_read > 0)
                    {
                        // 记录原始接收数据，用于诊断粘包问题
                        std::vector<uint8_t> raw_received(buffer.begin(), buffer.begin() + bytes_read);
                        std::string raw_hex = utils::bytesToHexString(raw_received);
                        static size_t recv_count = 0;
                        if (recv_count++ % 10 == 0) {  
                            RCLCPP_DEBUG(this->get_logger(), "[串口原始数据] 接收 %zu 字节: %s, 缓冲区已有 %zu 字节", 
                                       bytes_read, raw_hex.c_str(), receive_buffer_.size());
                        }
                        
                        // 使用互斥锁保护接收缓冲区
                        {
                            std::lock_guard<std::mutex> lock(receive_buffer_mutex_);
                            // 将接收的数据添加到缓冲区
                            receive_buffer_.insert(receive_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);
                        }

                        // 处理接收数据（注意：processSerialData 内部会修改 receive_buffer_）
                        if (protocol_) {
                            protocol_->processSerialData(receive_buffer_, [this](const std::string& hex_str) {
                                this->onSerialData(hex_str);
                            });
                        }
                    }
                } catch (const std::exception& ex) {
                    if (!serial_running_.load()) {
                        RCLCPP_INFO(this->get_logger(), "检测到退出信号，退出读取循环");
                        break;
                    }

                    // 检查是否是串口关闭导致的异常
                    if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
                        RCLCPP_WARN(this->get_logger(), "串口连接断开，准备重连");
                        continue;  // 跳转到重连逻辑
                    }

                    // 检查是否是IO错误，主动关闭串口触发重连
                    std::string error_msg = ex.what();
                    if (error_msg.find("Input/output error") != std::string::npos || 
                        error_msg.find("Bad file descriptor") != std::string::npos) {
                        try {
                            if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
                                serial_driver_->port()->close();
                            }
                        } catch (...) {}
                        continue;  // 跳转到重连逻辑
                    }

                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "串口读取异常: %s", ex.what());
                    std::this_thread::sleep_for(10ms);  // 减少睡眠时间，更快响应退出
                }
            }
            else
            {
                // 检查是否正在关闭，避免在关闭时阻塞
                if (!serial_running_.load())
                {
                    RCLCPP_INFO(this->get_logger(), "检测到关闭信号，退出读取线程");
                    break;
                }

                // 串口未打开,持续尝试重连
                reconnect_attempts++;
                RCLCPP_WARN(this->get_logger(), "串口断开，尝试重连第 %d 次", reconnect_attempts);

                // 发布错误事件
                if (!error_event_published || reconnect_attempts % 5 == 0) 
                {
                    auto error_code = std::make_shared<robot_interfaces::msg::MassageHeadErrorEvent>();
                    error_code->header.stamp = this->now();
                    error_code->header.frame_id = "massage_head_manager";
                    error_code->event_type = 3;  // 串口设备错误
                    error_code->event_msg = "串口设备断开";
                    error_pub_->publish(*error_code);
                    error_event_published = true;
                }

                // 设置重连标志
                is_reconnecting_.store(true);
                
                try {
                    // 创建新的IO上下文和串口驱动
                    io_context_ = std::make_shared<IoContext>();
                    serial_driver_ = std::make_shared<SerialDriver>(*io_context_);
                    auto config = SerialPortConfig(baudrate_, FlowControl::NONE, Parity::NONE, StopBits::ONE);
                    serial_driver_->init_port(device_name_, config);
                    serial_driver_->port()->open();
                    
                    if (serial_driver_->port()->is_open()) {
                        auto recovery_event = std::make_shared<robot_interfaces::msg::MassageHeadRecoveryEvent>();
                        recovery_event->header.stamp = this->now();
                        recovery_event->header.frame_id = "massage_head_manager";
                        recovery_event->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                        recovery_event->recovery_type = robot_interfaces::msg::MassageHeadRecoveryEvent::RECOVERY_TYPE_SERIAL_RECONNECT;
                        recovery_event->recovery_msg = "串口设备重连成功，通信已恢复";
                        recovery_pub_->publish(*recovery_event);
                        
                        RCLCPP_INFO(this->get_logger(), "恢复事件：%s", 
                                    recovery_event->recovery_msg.c_str());
                        
                        // 重置状态标志
                        reconnect_attempts = 0;
                        error_event_published = false;
                        is_reconnecting_.store(false);
                        
                        // 重连成功，继续读取循环
                        continue;
                    }
                } catch (const std::exception& ex) {
                    RCLCPP_WARN(this->get_logger(), "串口重连失败: %s", ex.what());
                }
                
                // 重连失败，清除重连标志并等待后继续尝试
                is_reconnecting_.store(false);
                int wait_time = std::min(reconnect_attempts, 5);
                std::this_thread::sleep_for(std::chrono::seconds(wait_time));
            }
        }
        catch (const std::exception& ex) {
            if (!serial_running_.load()) {
                RCLCPP_INFO(this->get_logger(), "检测到退出信号，退出读取循环");
                break;
            }

            RCLCPP_ERROR(this->get_logger(), "串口读取外层错误: %s", ex.what());
            std::this_thread::sleep_for(50ms);  // 减少睡眠时间
        }

        std::this_thread::sleep_for(10ms);
    }

    RCLCPP_INFO(this->get_logger(), "串口读取线程退出");
}

/**
 * @brief 处理JSON格式的电机控制
 * @param state 电机状态
 * @return bool 控制是否成功
 */
bool MassageHeadManageNode::handleMotorControlJson(int state)
{
    try {
        // 0=关闭, 1=正转(顺时针), 2=反转(逆时针)
        const char* state_str = (state == 0) ? "关闭" : (state == 1) ? "正转" : (state == 2) ? "反转" : "无效";
        RCLCPP_INFO(this->get_logger(), "电机控制: state=%d (%s)", state, state_str);

        std::vector<uint8_t> cmd;
        bool valid = true;

        // motor: 0=关闭, 1=正转, 2=反转
        switch(state) {
            case 0:
                cmd = protocol_->createCommandPacket(CommandType::SET_MOTOR_STATE, 0.0f);
                break;
            case 1:
                cmd = protocol_->createCommandPacket(CommandType::SET_MOTOR_STATE, 1.0f);
                break;
            case 2:
                cmd = protocol_->createCommandPacket(CommandType::SET_MOTOR_STATE, 2.0f);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "无效的电机状态值: %d (有效值: 0-关闭, 1-顺时针, 2-逆时针)", state);
                valid = false;
        }

        if (valid && !cmd.empty())
        {
            std::string desc = std::string("JSON电机控制:") + state_str;
            bool queued = enqueueControlCommand(cmd, desc);
            if (queued) {
                RCLCPP_INFO(this->get_logger(), "电机命令已入队");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "电机命令入队失败");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "电机命令生成失败");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "电机控制失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 处理JSON格式的LED控制
 * @param on LED开关状态
 * @return bool 控制是否成功
 */
bool MassageHeadManageNode::handleLedControlJson(bool on)
{
    try {
        RCLCPP_INFO(this->get_logger(), "LED控制: on=%s", on ? "true" : "false");

        auto cmd = protocol_->createCommandPacket(CommandType::SET_LED_STATE, on ? 1.0f : 0.0f);

        if (!cmd.empty())
        {
            std::string desc = std::string("JSON LED控制:") + (on ? "开启" : "关闭");
            bool queued = enqueueControlCommand(cmd, desc);
            if (queued) {
                RCLCPP_INFO(this->get_logger(), "LED命令已入队");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "LED命令入队失败");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "LED命令生成失败");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "LED控制失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 处理JSON格式的温度控制
 * @param temperature 目标温度（0表示关闭，>0表示设置温度）
 *                    V1协议: 30-60°C（实际温度）
 *                    V2协议: 10-75档（档位）
 * @return bool 控制是否成功
 */
bool MassageHeadManageNode::handleTempControlJson(float temperature)
{
    try {
        RCLCPP_INFO(this->get_logger(), "温度控制: temp=%.1f", temperature);

        // 检查是否为V2协议
        auto protocol_v2 = std::dynamic_pointer_cast<ProtocolV2>(protocol_);
        
        if (protocol_v2) {
            // V2协议：使用档位控制（10-75档）
            if (temperature > 0.0f && (temperature < 10.0f || temperature > 75.0f)) {
                RCLCPP_ERROR(this->get_logger(), "V2温度档位范围错误: %.1f档 (有效范围: 10-75)", temperature);
                return false;
            }

            std::vector<uint8_t> cmd;
            std::string desc;
            
            if (temperature > 0.0f) {
                // 设置温度档位
                uint8_t level = static_cast<uint8_t>(temperature);
                cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, level);
                desc = "JSON温度设置:" + std::to_string(level) + "档";
            } else {
                // 关闭温度
                cmd = protocol_v2->createPresetCommand(PresetCommand::TEMPERATURE, 0);
                desc = "JSON温度关闭";
            }

            if (!cmd.empty()) {
                bool queued = enqueueControlCommand(cmd, desc);
                if (queued) {
                    RCLCPP_INFO(this->get_logger(), "%s命令已入队", desc.c_str());
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "温度命令入队失败");
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "V2温度控制命令生成失败");
                return false;
            }
        } else {
            // V1协议：使用温度控制（30-60°C）
            if (temperature > 0.0f && (temperature < 30.0f || temperature > 60.0f)) {
                RCLCPP_ERROR(this->get_logger(), "V1温度范围错误: %.1f°C (有效范围: 30-60)", temperature);
                return false;
            }

            auto cmd = protocol_->createTemperatureSetCommand(temperature);

            if (!cmd.empty()) {
                std::string desc;
                if (temperature > 0.0f) {
                    desc = "JSON温度设置:" + std::to_string(static_cast<int>(temperature)) + "°C";
                } else {
                    desc = "JSON温度关闭";
                }
                
                bool queued = enqueueControlCommand(cmd, desc);
                if (queued) {
                    if (temperature > 0.0f) {
                        RCLCPP_INFO(this->get_logger(), "温度设置命令已入队: %.1f°C", temperature);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "温度关闭命令已入队");
                    }
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "温度命令入队失败");
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "V1温度控制命令生成失败");
                return false;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "温度控制失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 发送串口命令
 * @param data 要发送的命令数据
 * @return bool 发送成功返回true，失败返回false
 */
bool MassageHeadManageNode::sendSerialData(const std::vector<uint8_t>& data)
{
    // 加锁保护串口写操作，防止多个定时器和控制命令并发发送导致数据冲突
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    
    if (!serial_driver_ || !serial_driver_->port()->is_open())
    {
        RCLCPP_WARN(this->get_logger(), "串口未连接，无法发送数据");
        return false;
    }

    try {
        if (data.empty()) {
            RCLCPP_WARN(this->get_logger(), "无效的串口数据");
            return false;
        }

        // 直接发送字节数据
        size_t bytes_sent = serial_driver_->port()->send(data);
        std::string hex_str = utils::bytesToHexString(data);
        RCLCPP_DEBUG(this->get_logger(), "发送串口数据: %s (%zu bytes)", hex_str.c_str(), bytes_sent);
        return true;

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "发送串口数据失败: %s", ex.what());
        
        // 发布串口写入错误事件
        auto error_code = std::make_shared<robot_interfaces::msg::MassageHeadErrorEvent>();
        error_code->header.stamp = this->now();
        error_code->header.frame_id = "massage_head_manager";
        error_code->event_type = 3;  // 串口设备错误
        error_code->event_msg = "串口数据发送失败: " + std::string(ex.what());
        error_pub_->publish(*error_code);
        
        // 主动关闭串口以触发重连机制
        RCLCPP_WARN(this->get_logger(), "检测到串口IO错误，准备关闭串口以触发重连");
        try {
            if (serial_driver_ && serial_driver_->port()) {
                bool was_open = serial_driver_->port()->is_open();
                RCLCPP_INFO(this->get_logger(), "串口当前状态: %s", was_open ? "打开" : "关闭");
                if (was_open) {
                    serial_driver_->port()->close();
                    RCLCPP_WARN(this->get_logger(), "串口已关闭，等待读取线程检测并重连");
                }
            }
        } catch (const std::exception& close_ex) {
            RCLCPP_ERROR(this->get_logger(), "关闭串口时发生异常: %s", close_ex.what());
        }
        
        return false;
    }
}


/**
 * @brief 读取设备状态
 * 用于温度状态读取，只在用户操作后读取一次
 */
void MassageHeadManageNode::readStatusAfterCommand()
{
    if (!protocol_ || !serial_driver_ || !serial_driver_->port()->is_open()) {
        return;
    }

    try {
        // 检查设备是否在线
        if (current_handle_id_ == 0) {
            RCLCPP_DEBUG(this->get_logger(), "设备离线");
            return;
        }

        // 读取温度状态 - 温度有专用的读取命令(0x06)
        auto temp_cmd = protocol_->createTemperatureReadCommand();
        if (!temp_cmd.empty()) {
            sendSerialData(temp_cmd);
            RCLCPP_DEBUG(this->get_logger(), "发送温度读取命令");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "被动状态读取异常: %s", e.what());
    }
}

std::string MassageHeadManageNode::addPendingRequest(
    const std::string& serial_number,
    const nlohmann::json& params,
    std::shared_ptr<MassageHeadControlsrv::Response> response,
    bool is_json_request,
    const std::string& original_request_data)
{
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);

    // 生成唯一请求ID
    std::string request_id = "req_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

    // 创建挂起请求
    PendingRequest request;
    request.requested_serial_number = serial_number; 
    request.params = params;
    request.timestamp = std::chrono::steady_clock::now();
    request.expiry_time = request.timestamp + std::chrono::seconds(30); // 30秒超时
    request.request_id = request_id;
    request.response = response;
    request.is_json_request = is_json_request;
    request.original_request_data = original_request_data;

    // 添加到队列
    pending_requests_.push_back(request);

    RCLCPP_INFO(this->get_logger(), "添加挂起请求: serial_number=%s, request_id=%s, 队列大小=%zu",
                serial_number.c_str(), request_id.c_str(), pending_requests_.size());

    return request_id;
}



void MassageHeadManageNode::processPendingRequests()
{
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);

    if (pending_requests_.empty()) {
        return;
    }

    // 获取当前序列号
    if (current_serial_number_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "当前无按摩头连接，跳过挂起请求处理");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "检查挂起请求，当前序列号: %s, 队列大小: %zu",
                current_serial_number_.c_str(), pending_requests_.size());

    // 处理匹配的请求
    std::vector<PendingRequest> remaining_requests;
    std::set<std::string> processed_request_ids;

    for (const auto& request : pending_requests_) {
        // 检查是否已处理
        if (processed_request_ids.count(request.request_id) > 0) {
            RCLCPP_DEBUG(this->get_logger(), "跳过已处理的请求: %s", request.request_id.c_str());
            continue;
        }
        
        if (request.requested_serial_number == current_serial_number_) {
            // 序列号匹配，执行请求
            RCLCPP_INFO(this->get_logger(), "执行挂起请求: request_id=%s, serial_number=%s",
                        request.request_id.c_str(), request.requested_serial_number.c_str());

            try {
                // 重新创建JSON请求
                nlohmann::json full_request;
                full_request["properties"] = nlohmann::json::array();

                nlohmann::json config_item;
                config_item["serial_number"] = request.requested_serial_number;
                config_item["params"] = request.params;

                full_request["properties"].push_back(config_item);

                // 创建内部请求
                auto json_request = std::make_shared<MassageHeadControlsrv::Request>();
                json_request->command_params = full_request.dump();

                auto json_response = std::make_shared<MassageHeadControlsrv::Response>();
                handle_json_control_service(json_request, json_response);

                // 如果原始响应对象存在，更新结果
                if (request.response) {
                    request.response->success = json_response->success;
                    request.response->result = json_response->result;
                }

                // 标记为已处理
                processed_request_ids.insert(request.request_id);

                // 发布成功事件
                auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                event_msg->header.stamp = this->now();
                event_msg->header.frame_id = "massage_head_manager";
                event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                event_msg->is_attached = 1;  // 已安装
                event_msg->is_matched = true;
                event_msg->event_description = "挂起请求执行成功: " + request.request_id;
                event_pub_->publish(*event_msg);
                
                RCLCPP_INFO(this->get_logger(), "挂起请求执行成功: %s", request.request_id.c_str());

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "执行挂起请求失败: %s", e.what());

                // 失败则保留，等待重试
                remaining_requests.push_back(request);
                
                // 发布失败事件
                auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                event_msg->header.stamp = this->now();
                event_msg->header.frame_id = "massage_head_manager";
                event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                event_msg->is_attached = 1;  // 已安装但执行失败
                event_msg->is_matched = false;
                event_msg->event_description = "挂起请求执行失败: " + std::string(e.what());
                event_pub_->publish(*event_msg);
            }
        } else {
            // 功能码不匹配，保留在队列中
            remaining_requests.push_back(request);
        }
    }

    // 更新队列
    pending_requests_ = remaining_requests;
}



/*=================================================================================*/
void MassageHeadManageNode::cleanupExpiredPendingRequests()
{
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);

    if (pending_requests_.empty()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    std::vector<PendingRequest> valid_requests;

    for (const auto& request : pending_requests_) {
        if (now < request.expiry_time) {
            valid_requests.push_back(request);
        } else {
            // 更新响应
            if (request.response) {
                request.response->success = false;
                request.response->result = "请求超时";
            }

        }
    }

    pending_requests_ = valid_requests;
}

std::vector<std::map<std::string, std::string>> MassageHeadManageNode::getPendingRequestsStatus()
{
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);

    std::vector<std::map<std::string, std::string>> status_list;

    for (const auto& request : pending_requests_) {
        std::map<std::string, std::string> status;
        status["request_id"] = request.request_id;
        status["serial_number"] = request.requested_serial_number; 
        status["timestamp"] = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
            request.timestamp.time_since_epoch()).count());
        status["expiry_time"] = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
            request.expiry_time.time_since_epoch()).count());
        status["is_json_request"] = request.is_json_request ? "true" : "false";

        // 计算剩余时间
        auto now = std::chrono::steady_clock::now();
        auto remaining_seconds = std::chrono::duration_cast<std::chrono::seconds>(
            request.expiry_time - now).count();
        status["remaining_seconds"] = std::to_string(remaining_seconds);

        status_list.push_back(status);
    }

    return status_list;
}

void MassageHeadManageNode::pendingRequestsProcessorLoop()
{
    RCLCPP_INFO(this->get_logger(), "挂起请求处理线程启动");

    while (pending_processor_running_.load() && rclcpp::ok()) {
        try {
            // 清理过期请求
            cleanupExpiredPendingRequests();

            // 处理挂起请求
            processPendingRequests();

            // 分段休眠以便快速响应退出信号
            for (int i = 0; i < 20 && pending_processor_running_.load() && rclcpp::ok(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "挂起请求处理线程异常: %s", e.what());
            // 出错时也分段等待
            for (int i = 0; i < 50 && pending_processor_running_.load() && rclcpp::ok(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "挂起请求处理线程退出");
}

void MassageHeadManageNode::pendingRequestsTimerCallback()
{
    RCLCPP_DEBUG(this->get_logger(), "挂起请求定时器回调");

    // 清理过期请求
    cleanupExpiredPendingRequests();

    // 处理挂起请求
    processPendingRequests();
}

/**
 * @brief 发送TCP设置请求到arm_set_tcp服务
 * @param serial_number 按摩头序列号
 * @details 根据serial_number查询数据库获取TCP坐标，并发送到arm_set_tcp服务
 */

void MassageHeadManageNode::sendTcpSetRequestBySerialNumber(const std::string& serial_number)
{
    if (serial_number.empty()) {
        RCLCPP_WARN(this->get_logger(), "serial_number为空，无法查询TCP数据");
        
        // 发布初始化失败事件
        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
        event_msg->header.stamp = this->now();
        event_msg->header.frame_id = "massage_head_manager";
        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
        event_msg->is_attached = 2;  // 安装但初始化设置失败
        event_msg->is_matched = false;  // 默认false
        event_msg->event_description = "TCP设置失败: serial_number为空";
        event_pub_->publish(*event_msg);
        
        return;
    }

    std::map<std::string, TcpPosition> tcp_map;

    // 1. 优先从内存缓存（db_cache_）获取TCP数据
    auto cache_it = db_cache_.serial_to_info.find(serial_number);
    if (cache_it != db_cache_.serial_to_info.end()) {
        tcp_map = cache_it->second.tcp_positions;
        RCLCPP_INFO(this->get_logger(), "从内存缓存获取TCP数据: serial_number=%s, 共%zu个机器人配置",
                   serial_number.c_str(), tcp_map.size());
    } else if (database_manager_) {
        // 2. 内存缓存中没有，从数据库获取
        auto heads = database_manager_->getMassageHeadBySerialNumber(serial_number);
        if (!heads.empty() && !heads[0].id.empty()) {
            auto head = heads[0];
            tcp_map = DatabaseManager::parseTcpFromConfigData(head.config_data);
            RCLCPP_INFO(this->get_logger(), "从数据库获取TCP数据: serial_number=%s, 共%zu个机器人配置",
                       serial_number.c_str(), tcp_map.size());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "无法获取TCP数据：缓存和数据库都不可用");
        
        // 发布初始化失败事件
        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
        event_msg->header.stamp = this->now();
        event_msg->header.frame_id = "massage_head_manager";
        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
        event_msg->is_attached = 2;  // 安装但初始化设置失败
        event_msg->is_matched = false;  // 默认false
        event_msg->event_description = "TCP设置失败: 缓存和数据库都不可用";
        event_pub_->publish(*event_msg);
        
        return;
    }

    if (tcp_map.empty()) {
        RCLCPP_WARN(this->get_logger(), "未找到serial_number=%s对应的TCP数据", current_serial_number_.c_str());
        
        // 发布初始化失败事件
        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
        event_msg->header.stamp = this->now();
        event_msg->header.frame_id = "massage_head_manager";
        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
        event_msg->is_attached = 2;  // 安装但初始化设置失败
        event_msg->is_matched = false;  // 默认false
        event_msg->event_description = "TCP设置失败: 未找到对应的TCP数据";
        event_pub_->publish(*event_msg);
        
        return;
    }

    // 更新tcp_cache_用于快速访问
    tcp_cache_.is_valid = true;
    tcp_cache_.tcp_positions = tcp_map;

    // TCP设置前暂停离线检测
    if (protocol_) {
        protocol_->pauseOfflineDetection();
    }

    // 2. 遍历机器人配置，发送TCP设置请求
    for (const auto& [robot_name, tcp_pos] : tcp_map) {
        RCLCPP_INFO(this->get_logger(), "设置TCP: robot=%s, x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f",
                   robot_name.c_str(), tcp_pos.x, tcp_pos.y, tcp_pos.z, tcp_pos.rx, tcp_pos.ry, tcp_pos.rz);

        // 3. 检查客户端是否可用
        if (!arm_set_tcp_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "TCP服务不可用，跳过TCP设置: robot=%s", robot_name.c_str());
            
            
            if (protocol_) {
                protocol_->resumeOfflineDetection();
            }
            
            // 发布初始化失败事件
            auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
            event_msg->header.stamp = this->now();
            event_msg->header.frame_id = "massage_head_manager";
            event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
            event_msg->is_attached = 2;  // 安装但初始化设置失败
            event_msg->is_matched = false;  // 默认false
            event_msg->event_description = "TCP设置失败: TCP服务不可用(robot=" + robot_name + ")";
            event_pub_->publish(*event_msg);
            
            continue;
        }

        // 4. 构建服务请求
        auto request = std::make_shared<ArmSetTCPsrv::Request>();
        request->use_preset = false;  // 不使用预设偏移
        request->tcp_name = robot_name;
        
        // 填充TCP偏移数据（x, y, z, rx, ry, rz）
        request->tcp_offset[0] = static_cast<float>(tcp_pos.x);
        request->tcp_offset[1] = static_cast<float>(tcp_pos.y);
        request->tcp_offset[2] = static_cast<float>(tcp_pos.z);
        request->tcp_offset[3] = static_cast<float>(tcp_pos.rx);
        request->tcp_offset[4] = static_cast<float>(tcp_pos.ry);
        request->tcp_offset[5] = static_cast<float>(tcp_pos.rz);

        // 5. 异步发送请求
        // 注意：必须通过值捕获robot_name的副本，避免引用失效导致崩溃
        std::string robot_name_value = robot_name;
        auto result_future = arm_set_tcp_client_->async_send_request(request,
            [this, robot_name_value](rclcpp::Client<ArmSetTCPsrv>::SharedFuture future) {
                // 检查节点是否正在关闭
                if (node_shutting_down_.load()) {
                    return;
                }
                
                try {
                    // 等待响应，设置超时时间为2秒
                    auto status = future.wait_for(std::chrono::seconds(2));
                    
                    // 再次检查节点是否正在关闭
                    if (node_shutting_down_.load()) {
                        return;
                    }
                    
                    if (status == std::future_status::timeout) {
                        RCLCPP_ERROR(this->get_logger(), "TCP设置请求超时: robot=%s", robot_name_value.c_str());
                        
                        // 发布超时失败事件
                        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                        event_msg->header.stamp = this->now();
                        event_msg->header.frame_id = "massage_head_manager";
                        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                        event_msg->is_attached = 2;  // 安装但初始化设置失败
                        event_msg->is_matched = false;  // 默认false
                        event_msg->event_description = "TCP设置超时: robot=" + robot_name_value;
                        event_pub_->publish(*event_msg);
                        return;
                    }
                    
                    auto response = future.get();
                    bool this_success = false;
                    if (response->return_code == 1) {
                        RCLCPP_INFO(this->get_logger(), "TCP手动设置请求成功: robot=%s, msg=%s",
                                   robot_name_value.c_str(), response->return_msg.c_str());
                        this_success = true;
                    } else if (response->return_code == 0) {
                        RCLCPP_INFO(this->get_logger(), "TCP设置采用预设请求成功: robot=%s, msg=%s",
                                   robot_name_value.c_str(), response->return_msg.c_str());
                        this_success = true;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "TCP设置返回未知代码: robot=%s, code=%d, msg=%s",
                                   robot_name_value.c_str(), response->return_code, response->return_msg.c_str());
                    }

                    // TCP 设置结果处理
                    if (this_success) {
                        RCLCPP_INFO(this->get_logger(), "TCP设置成功");
                      
                        if (protocol_) {
                            protocol_->resumeOfflineDetection();
                        }
                        
                        // TCP设置成功后直接发布 is_attached=1 事件
                        RCLCPP_INFO(this->get_logger(), "TCP初始化成功，发布is_attached=1事件");
                        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                        event_msg->header.stamp = this->now();
                        event_msg->header.frame_id = "massage_head_manager";
                        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                        event_msg->is_attached = 1;  // 已安装且初始化成功
                        event_msg->is_matched = false;  // 默认false，只在请求匹配判断后才为true
                        event_msg->event_description = "设备安装成功，TCP初始化完成";
                        event_pub_->publish(*event_msg);
                    } else {
                      
                        if (protocol_) {
                            protocol_->resumeOfflineDetection();
                        }
                        
                        // TCP设置失败，立即发布 is_attached=2 事件
                        RCLCPP_ERROR(this->get_logger(), "TCP设置失败: robot=%s", robot_name_value.c_str());
                        auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                        event_msg->header.stamp = this->now();
                        event_msg->header.frame_id = "massage_head_manager";
                        event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                        event_msg->is_attached = 2;  // 安装但初始化设置失败
                        event_msg->is_matched = false;  // 默认false
                        event_msg->event_description = "TCP设置失败: robot=" + robot_name_value;
                        event_pub_->publish(*event_msg);
                    }
                } catch (const std::exception& e) {
                 
                    if (protocol_) {
                        protocol_->resumeOfflineDetection();
                    }
                    
                    RCLCPP_ERROR(this->get_logger(), "TCP设置请求异常: robot=%s, error=%s",
                                robot_name_value.c_str(), e.what());
                    
                    // 发布初始化失败事件
                    auto event_msg = std::make_shared<robot_interfaces::msg::MassageHeadAttachEvent>();
                    event_msg->header.stamp = this->now();
                    event_msg->header.frame_id = "massage_head_manager";
                    event_msg->serial_number = utils::hexSpacesToHyphens(current_serial_number_);
                    event_msg->is_attached = 2;  // 安装但初始化设置失败
                    event_msg->is_matched = false;  // 默认false
                    event_msg->event_description = "TCP设置异常: robot=" + robot_name_value + ", error=" + e.what();
                    event_pub_->publish(*event_msg);
                }
            });

    }
}

// ============================================================================
// 命令队列系统实现
// ============================================================================

void MassageHeadManageNode::initCommandProcessor()
{
    RCLCPP_INFO(this->get_logger(), "初始化命令处理器...");
    
    command_processor_running_ = true;
    command_processor_thread_ = std::thread(&MassageHeadManageNode::commandProcessorLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "命令处理器已启动");
}

void MassageHeadManageNode::stopCommandProcessor()
{
    RCLCPP_INFO(this->get_logger(), "停止命令处理器...");
    
    // 先设置退出标志
    command_processor_running_.store(false);
    
    // 唤醒等待中的处理线程
    {
        std::lock_guard<std::mutex> lock(command_queue_mutex_);
        command_queue_cv_.notify_all();
    }
    
    // 等待处理线程退出
    if (command_processor_thread_.joinable()) {
        // 最多等待1秒
        auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < 20; ++i) {
            {
                std::lock_guard<std::mutex> lock(command_queue_mutex_);
                command_queue_cv_.notify_all();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() > 1000) {
                break;
            }
        }
        
        try {
            command_processor_thread_.join();
            RCLCPP_INFO(this->get_logger(), "命令处理器线程已正常退出");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "命令处理器线程join失败: %s", e.what());
        }
    }
    
    // 清空队列
    clearCommandQueue();
    
    RCLCPP_INFO(this->get_logger(), "命令处理器已停止");
}

void MassageHeadManageNode::commandProcessorLoop()
{
    RCLCPP_INFO(this->get_logger(), "命令处理线程启动");
    
    while (command_processor_running_.load() && rclcpp::ok()) {
        CommandItem cmd_item;
        bool has_command = false;
        
        {
            std::unique_lock<std::mutex> lock(command_queue_mutex_);
            
            // 等待命令或退出信号，最多等待50ms以保持响应性
            bool signaled = command_queue_cv_.wait_for(lock, std::chrono::milliseconds(50), [this]() {
                return !command_queue_.empty() || !command_processor_running_.load();
            });
            
            // 再次检查退出条件
            if (!command_processor_running_.load()) {
                RCLCPP_DEBUG(this->get_logger(), "收到退出信号，命令处理线程准备退出");
                break;
            }
            
            if (signaled && !command_queue_.empty()) {
                cmd_item = command_queue_.top();
                command_queue_.pop();
                has_command = true;
            }
        }
        
        if (has_command) {
            // 检查命令是否过期（超过2秒的命令丢弃）
            auto now = std::chrono::steady_clock::now();
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - cmd_item.enqueue_time);
            
            if (age.count() > 2000) {
                RCLCPP_WARN(this->get_logger(), "命令已过期 (%ld ms): %s", 
                           age.count(), cmd_item.description.c_str());
                if (cmd_item.callback) {
                    cmd_item.callback(false);
                }
                continue;
            }
            
            // 发送命令
            bool success = sendSerialData(cmd_item.data);
            
            if (success) {
                RCLCPP_DEBUG(this->get_logger(), "命令发送成功: %s (队列剩余: %zu)", 
                            cmd_item.description.c_str(), getCommandQueueSize());
            } else {
                RCLCPP_WARN(this->get_logger(), "命令发送失败: %s", cmd_item.description.c_str());
            }
            
            // 执行回调
            if (cmd_item.callback) {
                cmd_item.callback(success);
            }
            
            // 命令间隔，避免过于密集
            std::this_thread::sleep_for(command_interval_);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "命令处理线程退出");
}

bool MassageHeadManageNode::enqueueControlCommand(const std::vector<uint8_t>& data, 
                                                   const std::string& description,
                                                   std::function<void(bool)> callback)
{
    if (data.empty()) {
        RCLCPP_WARN(this->get_logger(), "尝试入队空命令: %s", description.c_str());
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(command_queue_mutex_);
        
        // 限制队列大小，防止内存溢出
        if (command_queue_.size() >= 50) {
            RCLCPP_WARN(this->get_logger(), "命令队列已满，丢弃最旧命令");
            // 优先队列不支持直接删除，这里只是警告
        }
        
        CommandItem item;
        item.data = data;
        item.priority = CommandPriority::CONTROL_HIGH;
        item.description = description;
        item.enqueue_time = std::chrono::steady_clock::now();
        item.callback = callback;
        
        command_queue_.push(item);
    }
    
    // 通知处理线程
    command_queue_cv_.notify_one();
    
    RCLCPP_DEBUG(this->get_logger(), "控制命令已入队: %s (队列大小: %zu)", 
                description.c_str(), getCommandQueueSize());
    return true;
}

bool MassageHeadManageNode::enqueueStatusCommand(const std::vector<uint8_t>& data,
                                                  const std::string& description)
{
    if (data.empty()) {
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(command_queue_mutex_);
        
        // 队列较大时，跳过状态查询命令以优先处理控制命令
        if (command_queue_.size() >= 30) {
            RCLCPP_DEBUG(this->get_logger(), "队列较忙，跳过状态查询: %s", description.c_str());
            return false;
        }
        
        CommandItem item;
        item.data = data;
        item.priority = CommandPriority::STATUS_READ;
        item.description = description;
        item.enqueue_time = std::chrono::steady_clock::now();
        item.callback = nullptr;
        
        command_queue_.push(item);
    }
    
    command_queue_cv_.notify_one();
    return true;
}

bool MassageHeadManageNode::enqueueIdentifyCommand(const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(command_queue_mutex_);
        
        // 识别命令数量严格限制，避免堆积
        if (command_queue_.size() >= 20) {
            // 队列有积压时，跳过识别命令
            return false;
        }
        
        CommandItem item;
        item.data = data;
        item.priority = CommandPriority::IDENTIFY;
        item.description = "设备识别";
        item.enqueue_time = std::chrono::steady_clock::now();
        item.callback = nullptr;
        
        command_queue_.push(item);
    }
    
    command_queue_cv_.notify_one();
    return true;
}

void MassageHeadManageNode::clearCommandQueue()
{
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    
    // 清空优先队列
    while (!command_queue_.empty()) {
        command_queue_.pop();
    }
    
    RCLCPP_INFO(this->get_logger(), "命令队列已清空");
}

size_t MassageHeadManageNode::getCommandQueueSize()
{
    std::lock_guard<std::mutex> lock(command_queue_mutex_);
    return command_queue_.size();
}

int main(int argc, char **argv)
{
    // 授权验证
    auto license_result = license_manager::check_system_license();
    if (!license_result.is_valid) {
        std::cerr << "[MassageHeadManager] 授权验证失败: " << license_result.message << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MassageHeadManageNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) 
    {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();

    std::cout << "按摩头管理节点已正常退出" << std::endl;
    return 0;
}
