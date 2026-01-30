/**
 * @brief
 * 协议解析实现，继承自 IProtocol
 * 管理多个按摩头协议实例
 * 通过 createCommandPacket 方法创建不同类型的命令包
 * @return
 * 指令生成（电机、LED、温度设置、累计用时读取）
 * 数据包解析
 * 打印解析结果
 *
 * @details
 * 要根据协议注意功能码或者命令码的  payload[]  位置
 *
 * rclcpp::Logger 打印
 *
 * 新协议格式：
 * 发送格式：5A A5 05 82 10 FUNC 00 PAYLOAD
 * 返回格式：5A A5 06 83 10 FUNC 01 00 PAYLOAD
 *
 * 
 * │   │   │   │   │   │    │   └────── 载荷数据（PAYLOAD）
 * │   │   │   │   │   │    └────────── 固定值 0x00
 * │   │   │   │   │   └─────────────── 功能码（FUNC）
 * │   │   │   │   └─────────────────── 固定值 0x10
 * │   │   │   └─────────────────────── 固定值 0x82（发送标识）
 * │   │   └─────────────────────────── 数据长度 0x05（固定长度）
 * │   └─────────────────────────────── 固定头 0xA5
 * └─────────────────────────────────── 固定头 0x5A
 * 
 * ================返回包结构（8字节）==============
 * │   │   │   │   │   │    │   │   └── 载荷数据（PAYLOAD）
 * │   │   │   │   │   │    │   └────── 固定值 0x00
 * │   │   │   │   │   │    └────────── 固定值 0x01
 * │   │   │   │   │   └─────────────── 功能码（FUNC）
 * │   │   │   │   └─────────────────── 固定值 0x10
 * │   │   │   └─────────────────────── 固定值 0x83（返回标识）
 * │   │   └─────────────────────────── 数据长度 0x06（固定长度）
 * │   └─────────────────────────────── 固定头 0xA5
 * └─────────────────────────────────── 固定头 0x5A
 * 
 * 
 * 
 * 手柄功能说明：
 * 1. 指压手柄：识别、累计用时
 * 2. 滚轮手柄：识别、累计用时
 * 3. 负压手柄：识别、累计用时、温度设置和检测
 * 4. 冲击波手柄：识别、累计用时
 * 5. 旋转手柄：识别、累计用时、温度功能、电机正反转停止功能
 *
 * 功能码映射：
 * 0x01: 手柄识别
 * 0x05: 温度设置
 * 0x06: 温度读取
 * 0x07: 电机控制
 * 0x08: LED控制
 * 0x09: 累计用时读取
 */


// ProtocolV1
#include "massage_head_manager/protocol_v1.hpp"
#include "massage_head_manager/protocol_utils.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <map>
#include <chrono>

namespace massage_head_manager {
ProtocolV1::ProtocolV1(const rclcpp::Logger &logger, rclcpp::Clock::SharedPtr clock,
                       std::shared_ptr<DatabaseManager> db_manager)
: logger_(logger), clock_(clock), db_manager_(db_manager)
{
    // 初始状态为未连接
    is_device_online_ = false;
    
    if (db_manager_) {
        RCLCPP_INFO(logger_, "ProtocolV1: 数据库管理器注入成功");
    } else {
        RCLCPP_WARN(logger_, "ProtocolV1: 数据库管理器未注入，某些功能将受限");
    }
    
    RCLCPP_INFO(logger_, "ProtocolV1: 协议初始化完成");
}

// 状态更新：由上层调用以反映预期状态或立即更新内部状态
void ProtocolV1::updateMotorState(uint8_t state)
{
    current_motor_state_ = state;
    RCLCPP_DEBUG(logger_, "ProtocolV1: updateMotorState -> 0x%02X", state);
}

void ProtocolV1::updateLedState(bool on)
{
    current_led_state_ = on;
    RCLCPP_DEBUG(logger_, "ProtocolV1: updateLedState -> %s", on ? "ON" : "OFF");
}

void ProtocolV1::updateTempState(bool enable)
{
    if (!enable) {
        current_temp_set_ = 0.0f;
    } else if (current_temp_set_ <= 0.0f) {
        current_temp_set_ = 42.0f;
    }
    RCLCPP_DEBUG(logger_, "ProtocolV1: updateTempState -> %s (set=%.1f)", enable ? "ENABLED" : "DISABLED", current_temp_set_);
}

void ProtocolV1::updateTempStateSet(float temperature)
{
    if (temperature <= 0.0f) {
        current_temp_set_ = 0.0f;
    } else {
        // clamp to valid range
        if (temperature < 30.0f) temperature = 30.0f;
        if (temperature > 60.0f) temperature = 60.0f;
        current_temp_set_ = temperature;
    }
    RCLCPP_DEBUG(logger_, "ProtocolV1: updateTempStateSet -> %.1f", current_temp_set_);
}

// 重置所有状态变量
void ProtocolV1::resetStatus()
{
    current_handle_id_ = 0;
    current_handle_name_.clear();
    current_temp_set_ = 0.0f;
    current_temp_actual_ = 0.0f;
    current_motor_state_ = 0;
    current_led_state_ = false;
    current_usage_time_ = 0;
    is_device_online_ = false;
    no_response_count_ = 0;  // 重置计数器
}

// 更新设备在线状态
void ProtocolV1::updateLastPacketTime()
{
    is_device_online_ = true;
}

// 更新心跳状态
void ProtocolV1::updateHeartbeatTime()
{
    is_device_online_ = true;
}

// 检查设备是否在线
bool ProtocolV1::checkDeviceOnline()
{
    return is_device_online_;
}

// 增加未响应计数
void ProtocolV1::incrementNoResponseCount()
{
    // 检查离线检测是否启用
    if (!offline_detection_enabled_) {
        // 检查暂停时长
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - offline_detection_pause_time_).count();
        
        if (elapsed < OFFLINE_DETECTION_PAUSE_DURATION_MS) {
            // 仍在暂停期内，不增加计数
            return;
        } else {
            // 暂停期结束，自动恢复
            resumeOfflineDetection();
        }
    }
    
    no_response_count_++;
}

// 重置计数器
void ProtocolV1::resetNoResponseCount()
{
    no_response_count_ = 0;
}

// 检查是否应该清除设备
bool ProtocolV1::shouldClearDevice()
{
    // 离线检测禁用时不清除设备
    if (!offline_detection_enabled_) {
        return false;
    }
    
    return no_response_count_ >= MAX_NO_RESPONSE_COUNT;
}

// 暂停离线检测
void ProtocolV1::pauseOfflineDetection()
{
    offline_detection_enabled_ = false;
    offline_detection_pause_time_ = std::chrono::steady_clock::now();
    resetNoResponseCount();  // 暂停时重置计数器
}

// 恢复离线检测
void ProtocolV1::resumeOfflineDetection()
{
    offline_detection_enabled_ = true;
    resetNoResponseCount();
}



// V1 校验
bool ProtocolV1::validatePacketV1(const std::vector<uint8_t> &data)
{
    // 最小长度检查：返回包至少9字节
    if (data.size() < 9) return false;

    // 固定头检查
    if (data[0] != 0x5A || data[1] != 0xA5) return false;


    if (data[3] == 0x82) 
    {
        return false;  // 拒绝命令回显
    }


    // 返回包格式验证：5A A5 06 83 10 FUNC 01 00 PAYLOAD（温度、电机、LED等）
    if (data[2] == 0x06 && data[3] == 0x83 && data[4] == 0x10 && data[5] >= 0x01 && data[5] <= 0x09 && data[6] == 0x01 && data[7] == 0x00)
    {
        return true;
    }

    // 累计用时返回包格式：5A A5 06 83 10 09 01 高位 低位（注意：没有0x00字节）
    if (data[2] == 0x06 && data[3] == 0x83 && data[4] == 0x10 && data[5] == 0x09 && data[6] == 0x01 && data.size() >= 9)
    {
        return true;
    }
    return false;
}


// 构造发送包（V1）: [5A A5 05 82 10 FUNC 00 PAYLOAD]
std::vector<uint8_t> ProtocolV1::buildBasePacketV1(uint8_t func_code, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> pkt;
    pkt.reserve(7 + payload.size());  // 预留空间避免重新分配
    pkt.insert(pkt.end(), {0x5A, 0xA5, 0x05, 0x82, 0x10, func_code, 0x00});

    if (!payload.empty()) {
        pkt.insert(pkt.end(), payload.begin(), payload.end());
    }
    return pkt;
}

// cmd: 指令类型（SET_TEMPERATURE / SET_MOTOR_STATE / SET_LED_STATE）
// value: 对应的数值（温度数值、电机状态、LED开关）
std::vector<uint8_t> ProtocolV1::createCommandPacket(CommandType cmd, float value)
{
    std::vector<uint8_t> packet;
 
    uint8_t payload_value = 0;

    switch (cmd) {
        case CommandType::SET_MOTOR_STATE:
            /*
             * 电机控制命令:
             * 0x5A 0xA5 0x05 0x82 0x10 0x07 0x00 [motor_state]
             * 其中 [motor_state]: 0x00=关闭, 0x01=正转, 0x02=反转
             * 例如反转命令: 5A A5 05 82 10 07 00 02
             */
            payload_value = static_cast<uint8_t>(value);
            packet = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0x07, 0x00, payload_value};
            break;
        case CommandType::SET_TEMPERATURE:
            // 温度设置：30-60度，步进0.5度
            if (value <= 0) 
            {
                payload_value = 0xFF; // 关闭温度
            } else 
            {
                // 转换公式：温度值 = (目标温度 - 30) × 2
                payload_value = static_cast<uint8_t>((value - 30.0f) * 2.0f);
                // 限制在有效范围内 0x00-0x3C
                if (payload_value > 0x3C) payload_value = 0x3C;
            }
            packet = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0x05, 0x00, payload_value};
            break;
        case CommandType::SET_LED_STATE:
            // LED控制：01亮/00灭
            payload_value = static_cast<uint8_t>(value > 0 ? 0x01 : 0x00);
            packet = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0x08, 0x00, payload_value};
            break;
        default:
            RCLCPP_WARN(logger_, "未知指令类型: 0x%02X", static_cast<uint8_t>(cmd));
            return {};
    }
    RCLCPP_DEBUG(logger_, "生成命令包: %s", utils::bytesToHexString(packet).c_str());
    return packet;
}



// 解析包 -> 返回 BaseParseResult
BaseParseResult ProtocolV1::parsePacket(const std::vector<uint8_t> &data)
{
    
    BaseParseResult res;
    res.raw_data = utils::bytesToHexString(data);
    
    if (!validatePacketV1(data)) {
        res.success = false;
        res.type = BaseParseResult::MessageType::OTHER;
        RCLCPP_DEBUG(logger_, "ProtocolV1: invalid packet: %s", res.raw_data.c_str());
        return res;
    }
    // 更新最后收到数据包的时间,用于超时检测
    updateLastPacketTime();
    
    // 构造serial_number用于数据库查询
    res.serial_number = utils::bytesToDatabaseHexString(data);

    // func code->索引5，payload->索引8
    uint8_t func = data[5];
    

    res.success = true;
    
    // 收到任何有效数据包，重置未响应计数器
    resetNoResponseCount();

    switch (func) {
          case 0x01:  // HANDLE IDENTIFY
              res.type = BaseParseResult::MessageType::HANDLE_INFO;
              // 返回包格式：5A A5 06 83 1001 01 00 手柄号
              if (data.size() >= 9) {
                  uint8_t payload = data[8];
                  res.handle_id = static_cast<int>(payload);

                  // ========== 注释原因：避免与massage_head_manager层的db_cache_重复 ==========
                  // 数据库查询和缓存功能已在massage_head_manager层的loadDatabaseCache()中实现
                  // Protocol层只负责协议解析，不再重复查询数据库
                  // ========================================================================
                  
                  // 使用简单的默认名称（上层会通过db_cache_获取正确名称）
                  res.handle_name = "手柄_" + std::to_string(payload);
                  current_handle_name_ = res.handle_name;
                  
                  // 只在按摩头发生变化时打印识别信息
                  if (last_handle_id_ != static_cast<uint8_t>(payload)) {
                      RCLCPP_INFO(logger_, "识别按摩头: ID=0x%02X", payload);
                  }
                  
                  /* ===== 已注释：重复的数据库查询和缓存逻辑 =====
                  // 从缓存或数据库获取按摩头信息（通过serial_number查询）
                  if (db_manager_) {
                      MassageHeadCache cached_data;
                      bool found_in_cache = false;
                      
                      // 首先检查缓存
                      {
                          std::lock_guard<std::mutex> lock(cache_mutex_);
                          auto cache_it = massage_head_cache_.find(res.serial_number);
                          if (cache_it != massage_head_cache_.end() && cache_it->second.is_cached) {
                              cached_data = cache_it->second;
                              found_in_cache = true;
                              RCLCPP_DEBUG(logger_, "从缓存中读取按摩头信息: serial_number=%s", res.serial_number.c_str());
                          }
                      }
                      
                      if (found_in_cache) {
                          // 使用缓存数据
                          res.handle_name = cached_data.name;
                          current_handle_name_ = cached_data.name;
                          
                          // 只在按摩头发生变化时打印识别信息
                          if (last_handle_id_ != static_cast<uint8_t>(payload)) {
                              RCLCPP_INFO(logger_, "识别按摩头(缓存): %s", cached_data.name.c_str());
                          }
                      } else {
                          // 缓存未命中，查询数据库
                          auto heads = db_manager_->getMassageHeadBySerialNumber(res.serial_number);
                          if (!heads.empty()) {
                              const auto& head = heads[0];
                              res.handle_name = head.name;
                              current_handle_name_ = head.name;
                              
                              // 解析并缓存数据
                              MassageHeadCache new_cache;
                              new_cache.name = head.name;
                              new_cache.description = head.description;
                              new_cache.tcp_map = db_manager_->parseTcpFromConfigData(head.config_data);
                              new_cache.payload = db_manager_->parsePayloadFromConfigData(head.config_data);
                              new_cache.is_cached = true;
                              
                              // 存入缓存
                              {
                                  std::lock_guard<std::mutex> lock(cache_mutex_);
                                  massage_head_cache_[res.serial_number] = new_cache;
                              }
                              
                              if (new_cache.tcp_map.empty()) {
                                  RCLCPP_WARN(logger_, "未能解析 TCP 偏移量，使用默认值 0.0");
                              }

                              // 只在按摩头发生变化时打印识别信息
                              if (last_handle_id_ != static_cast<uint8_t>(payload)) {
                                  RCLCPP_INFO(logger_, "识别按摩头(数据库): %s, 已缓存", head.name.c_str());
                              }
                          } else {
                              res.handle_name = "未知手柄";
                              current_handle_name_ = "未知手柄";
                              RCLCPP_WARN(logger_, "数据库中未找到 serial_number=%s 的按摩头", res.serial_number.c_str());
                          }
                      }
                  }
                  ===== 已注释结束 ===== */
                  
                  current_handle_id_ = static_cast<uint8_t>(payload);

                  // 更新last_handle_id_用于下次比较
                  last_handle_id_ = current_handle_id_;
                  
                  // 更新心跳时间
                  updateHeartbeatTime();
              }
              resetNoResponseCount();
              break;

          case 0x05: // TEMP SET
              res.type = BaseParseResult::MessageType::TEMP_SET;
              // 返回包格式：5A A5 06 83 1005 01 00 温度档位
              // 温度档位 0x00(30度)~0x3C(60度)，共60档，每档0.5度
              if (data.size() >= 9) {
                  uint8_t payload = data[8];
                  if (payload == 0xFF) {
                      res.temp_value = 0.0f; // 关闭温度设置
                      current_temp_set_ = 0.0f;
                  } else if (payload > 0x3C) {
                    // 温度档位范围检查：有效范围 0x00-0x3C (0-60档)
                      RCLCPP_WARN(logger_, "温度设置异常: payload=0x%02X 超出有效档位范围(0x00-0x3C), 原始数据: %s", 
                                  payload, res.raw_data.c_str());  
                      res.success = false;
                  } else {
                      // 档位转温度：temp = 30 + 档位 * 0.5
                      res.temp_value = 30.0f + static_cast<float>(payload) * 0.5f;
                      current_temp_set_ = res.temp_value;
                  }
              }
              resetNoResponseCount();
              break;

          case 0x06: // TEMP READ
              res.type = BaseParseResult::MessageType::TEMP_READ;
              // 返回包格式：5A A5 06 83 1006 01 00 温度档位
              // 温度档位 0x00(30度)~0x3C(60度)，共60档，每档0.5度
              if (data.size() >= 9) {
                  uint8_t payload = data[8];
                  
                  // 温度档位范围检查：有效范围 0x00~0x3C (0-60档，对应30-60°C)
                  // payload超出范围可能是数据错误或串口通信异常
                  if (payload > 0x3C) {
                    //   RCLCPP_WARN(logger_, "温度读取异常: payload=0x%02X 超出有效档位范围(0x00-0x3C), 原始数据: %s", 
                    //               payload, res.raw_data.c_str());
                      res.success = false;
                      break; 
                  }
                  
                  // 档位转温度：temp = 30 + 档位 * 0.5
                  res.temp_value = 30.0f + static_cast<float>(payload) * 0.5f;

                  // 记录温度变化
                  if (std::abs(current_temp_actual_ - res.temp_value) > 0.1f) {
                      RCLCPP_DEBUG(logger_, "温度变化 %.1f°C -> %.1f°C",
                                  current_temp_actual_, res.temp_value);
                  }

                  current_temp_actual_ = res.temp_value;

                  // 更新心跳时间 
                  updateHeartbeatTime();
              }
              resetNoResponseCount();
              break;

          case 0x07: // MOTOR STATE
              res.type = BaseParseResult::MessageType::MOTOR_STATE;
              // 返回包格式：5A A5 06 83 1007 01 00 电机状态
              if (data.size() >= 9) {
                  uint8_t payload = data[8];
                  res.motor_state = payload;
                  current_motor_state_ = payload;

                  // 更新心跳时间 
                  updateHeartbeatTime();
              }
              resetNoResponseCount();
              break;

          case 0x08: // LED STATE
              res.type = BaseParseResult::MessageType::LED_STATE;
              // 返回包格式：5A A5 06 83 1008 01 00 LED状态
              if (data.size() >= 9) {
                  uint8_t payload = data[8];
                  res.led_state = (payload == 0x01);
                  current_led_state_ = res.led_state;
              }
              resetNoResponseCount();
              break;

          case 0x09: // 累计用时
              res.type = BaseParseResult::MessageType::USAGE_TIME;
              // 返回包格式：5A A5 06 83 10 09 01 分钟高位 分钟低位
              // 索引: 0  1  2  3  4  5  6  7       8
              if (data.size() >= 9) {
                  uint8_t high_byte = data[7];  // 高位字节在索引7
                  uint8_t low_byte = data[8];   // 低位字节在索引8
                  res.usage_time = (static_cast<uint16_t>(high_byte) << 8) | low_byte;
                  current_usage_time_ = res.usage_time;
              }
              resetNoResponseCount();
              break;

          default:
              res.type = BaseParseResult::MessageType::OTHER;
              resetNoResponseCount();
              break;
      }

      return res;
  }


// 打印解析结果
void ProtocolV1::printParseResult(const BaseParseResult &result)
{
    if (!result.success) {
        RCLCPP_DEBUG(logger_, "数据解析失败: %s", result.raw_data.c_str());
        return;
    }

    switch (result.type) {
        case BaseParseResult::MessageType::HANDLE_INFO:
            // 只在手柄ID变化时打印INFO
            if (result.handle_id != last_handle_id_) {
                RCLCPP_INFO(logger_, "手柄识别 -> id=%d name=%s",
                            result.handle_id, result.handle_name.c_str());
                last_handle_id_ = result.handle_id;
            } else {
                RCLCPP_DEBUG(logger_, "手柄识别 -> id=%d name=%s",
                            result.handle_id, result.handle_name.c_str());
            }
            break;

        case BaseParseResult::MessageType::TEMP_SET:
            RCLCPP_INFO(logger_, "温度设置 -> %.1f°C", result.temp_value);
            break;

        case BaseParseResult::MessageType::TEMP_READ:
            // 只在温度实际值变化超过0.5°C时打印INFO
            // 同时增加时间间隔限制，避免短时间内重复打印
            {
                static auto last_temp_log_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_temp_log_time).count();
                
                // 温度变化超过0.5°C 且距离上次打印超过1秒
                if (std::abs(result.temp_value - last_temp_actual_) > 0.5f && elapsed > 1000) {
                    // RCLCPP_INFO(logger_, "温度读取 -> %.1f°C", result.temp_value);
                    last_temp_actual_ = result.temp_value;
                    last_temp_log_time = now;
                } else {
                    RCLCPP_DEBUG(logger_, "温度读取 -> %.1f°C", result.temp_value);
                }
            }
            break;

        case BaseParseResult::MessageType::MOTOR_STATE:
            {
                const char *s = (result.motor_state == 0 ? "STOP" : (result.motor_state == 1 ? "CW" : "CCW"));
                // 只在电机状态变化时打印INFO
                if (result.motor_state != last_motor_state_) {
                    RCLCPP_INFO(logger_, "电机状态 -> %s ", s);
                    last_motor_state_ = result.motor_state;
                } else {
                    RCLCPP_DEBUG(logger_, "电机状态 -> %s ", s);
                }
            }
            break;

        case BaseParseResult::MessageType::LED_STATE:
            // 只在LED状态变化时打印INFO
            if (result.led_state != last_led_state_) {
                RCLCPP_INFO(logger_, "LED状态 -> %s", result.led_state ? "ON" : "OFF");
                last_led_state_ = result.led_state;
            } else {
                RCLCPP_DEBUG(logger_, "LED状态 -> %s", result.led_state ? "ON" : "OFF");
            }
            break;

        case BaseParseResult::MessageType::USAGE_TIME:
            // 累计用时变化通常较慢，每次打印INFO
            // RCLCPP_INFO(logger_, "累计用时 -> %u 分钟 (%.1f 小时)", result.usage_time, result.usage_time / 60.0f);
            break;

        default:
            RCLCPP_DEBUG(logger_, "其他 -> %s", result.raw_data.c_str());
            break;
    }
}

// 命令生成
std::vector<uint8_t> ProtocolV1::createIdentifyCommand(uint8_t handle_id)
{
    return buildBasePacketV1(0x01, {handle_id});
}

std::vector<uint8_t> ProtocolV1::createTemperatureSetCommand(float temperature)
{
    uint8_t temp_value;
    
    if (temperature <= 0.0f) {
        // 关闭加热
        temp_value = 0xFF;
        current_temp_set_ = 0.0f;
        RCLCPP_INFO(logger_, "关闭加热功能");
    } else {
        // 启用加热：温度范围限制 30-60°C
        if (temperature < 30.0f) temperature = 30.0f;
        if (temperature > 60.0f) temperature = 60.0f;
        
        // 计算档位：根据协议 0x00(30度)~0x3C(60度)，共60档，每档0.5度
        // 公式：档位 = (温度 - 30) * 2
        temp_value = static_cast<uint8_t>((temperature - 30.0f) * 2.0f);
        // 限制在有效范围 0x00-0x3C
        if (temp_value > 0x3C) temp_value = 0x3C;
        current_temp_set_ = temperature;
        RCLCPP_INFO(logger_, "ProtocolV1: 设置温度 %.1f°C ", temperature);
    }
    
    return buildBasePacketV1(0x05, {temp_value});
}

std::vector<uint8_t> ProtocolV1::createMotorControlCommand(uint8_t motor_state)
{
    return buildBasePacketV1(0x07, {motor_state});
}

std::vector<uint8_t> ProtocolV1::createLedControlCommand(bool led_on)
{
    // LED状态将在收到硬件响应后更新，确保状态准确性
    // 临时保存目标状态，但不立即更新current_led_state_
    return buildBasePacketV1(0x08, {static_cast<uint8_t>(led_on ? 0x01 : 0x00)});
}

std::vector<uint8_t> ProtocolV1::createUsageTimeReadCommand()
{
    return buildBasePacketV1(0x09, {0x01});
}

std::vector<uint8_t> ProtocolV1::createTemperatureReadCommand()
{
    // 发送格式：5A A5 05 82 10 06 00 01 (功能码0x06, 读取温度)
    return buildBasePacketV1(0x06, {0x01});
}



InternalStatus ProtocolV1::getCurrentStatus()
{
    // 检查设备是否在线 
    checkDeviceOnline();

    InternalStatus status;
    status.motor_state = current_motor_state_;
    status.led_state = current_led_state_;
    status.temp_set = current_temp_set_;
    status.temp_actual = current_temp_actual_;
    status.usage_time = current_usage_time_;

    return status;
}


std::string ProtocolV1::getFunctionCodeBySerialNumber(const std::string& serial_number) {
    RCLCPP_DEBUG(logger_, "通过序列号查询功能码: '%s'", serial_number.c_str());
    
    // ========== 注释原因：避免与massage_head_manager层的db_cache_重复 ==========
    // 数据库查询功能已在massage_head_manager层统一处理
    // Protocol层不再重复查询，返回占位符供上层使用
    // ========================================================================
    
    return "手柄设备";  // 返回通用名称，上层会通过db_cache_获取正确名称
    
    /* ===== 已注释：重复的数据库查询逻辑 =====
    if (!db_manager_) {
        RCLCPP_ERROR(logger_, "数据库管理器未初始化，无法查询功能码");
        return "未知手柄";
    }
    
    // 首先检查缓存
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        auto cache_it = massage_head_cache_.find(serial_number);
        if (cache_it != massage_head_cache_.end() && cache_it->second.is_cached) {
            RCLCPP_DEBUG(logger_, "从缓存中获取按摩头名称: %s", cache_it->second.name.c_str());
            return cache_it->second.name;
        }
    }
    
    // 缓存未命中，从数据库查询按摩头信息
    auto heads = db_manager_->getMassageHeadBySerialNumber(serial_number);
    
    if (!heads.empty()) {
        const auto& head = heads[0];
        RCLCPP_INFO(logger_, "查询到按摩头: serial=%s, name=%s",
                   serial_number.c_str(), head.name.c_str());
        return head.name;
    }
    
    RCLCPP_WARN(logger_, "数据库中未找到serial_number='%s'的按摩头", serial_number.c_str());
    return "未知手柄";
    ===== 已注释结束 ===== */
}

std::string ProtocolV1::getSerialNumberFromPacket(const std::vector<uint8_t>& data) {
    // 只提取设备识别包的前9个字节作为序列号
    if (data.size() >= 9) {
        std::vector<uint8_t> id_data(data.begin(), data.begin() + 9);
        std::string serial = utils::bytesToDatabaseHexString(id_data);
        RCLCPP_DEBUG(logger_, "生成的序列号: %s", serial.c_str());
        return serial;
    }
    std::string serial = utils::bytesToDatabaseHexString(data);
    RCLCPP_DEBUG(logger_, "生成的序列号: %s", serial.c_str());
    return serial;
}

// 处理接收到的串口数据 - 提取完整数据包
void ProtocolV1::processSerialData(std::vector<uint8_t>& data, std::function<void(const std::string&)> callback)
{
    size_t processed_bytes = 0;

    if (!data.empty()) {
        std::string raw_hex = utils::bytesToHexString(data);
        RCLCPP_DEBUG(logger_, "processSerialData: 收到原始数据 [%zu字节]: %s", data.size(), raw_hex.c_str());
    }

    while (processed_bytes < data.size()) {
        // 寻找有效包头 0x5A 0xA5
        bool found_header = false;
        size_t header_pos = processed_bytes;

        for (; header_pos < data.size() - 1; header_pos++) {
            if (data[header_pos] == 0x5A && data[header_pos + 1] == 0xA5) {
                found_header = true;
                break;
            }
        }

        if (!found_header) {
            // 没有找到有效包头，清除已处理的数据
            data.erase(data.begin(), data.begin() + processed_bytes);
            return;
        }

        // 移动到包头位置
        processed_bytes = header_pos;
        RCLCPP_DEBUG(logger_, "找到包头在位置 %zu, 剩余数据长度 %zu", header_pos, data.size() - header_pos);

        // 检查是否有足够的数据来确定包类型和功能码
        if (data.size() - processed_bytes < 6) {
            // 数据不足，等待更多数据
            data.erase(data.begin(), data.begin() + processed_bytes);
            return;
        }

        uint8_t packet_type = data[processed_bytes + 2];  // 0x05: 发送包, 0x06: 返回包
        uint8_t func_code = data[processed_bytes + 5];    // 功能码

        // 根据协议文档确定包长度
        size_t expected_length = 0;

        switch (func_code) {
            case 0x01:  // 设备识别
                expected_length = 9;  // 识别包都是9字节，无论发送还是返回
                break;
            case 0x02:  // 温度设置
                expected_length = (packet_type == 0x05) ? 10 : 9;
                break;
            case 0x03:  // 温度读取
                expected_length = (packet_type == 0x05) ? 9 : 10;
                break;
            case 0x04:  // 电机控制
                expected_length = (packet_type == 0x05) ? 10 : 9;
                break;
            case 0x05:  // 电机状态读取
                expected_length = (packet_type == 0x05) ? 9 : 10;
                break;
            case 0x06:  // LED控制
                expected_length = (packet_type == 0x05) ? 10 : 9;
                break;
            case 0x07:  // LED状态读取
                expected_length = (packet_type == 0x05) ? 9 : 10;
                break;
            case 0x08:  // 设备启停控制
                expected_length = (packet_type == 0x05) ? 10 : 9;
                break;
            case 0x09:  // 累积用时读取
                expected_length = (packet_type == 0x05) ? 9 : 10;
                break;
            default:
                // 未知功能码，跳过这个包头
                processed_bytes += 2;
                continue;
        }

        // 检查是否有足够的数据
        if (data.size() - processed_bytes < expected_length) {
            // 数据不完整，等待更多数据
            data.erase(data.begin(), data.begin() + processed_bytes);
            return;
        }

        // 提取完整数据包
        std::vector<uint8_t> packet(data.begin() + processed_bytes,
                                  data.begin() + processed_bytes + expected_length);

        // 记录提取数据包的原始内容，用于诊断
        std::string packet_hex = utils::bytesToHexString(packet);
        RCLCPP_DEBUG(logger_, "提取数据包: 期望长度=%zu, 实际提取长度=%zu, 起始位置=%zu, 内容=%s",
                    expected_length, packet.size(), processed_bytes, packet_hex.c_str());

        // 验证包边界：检查下一个位置是否是新包头(如果有足够数据)
        size_t next_pos = processed_bytes + expected_length;
        if (data.size() > next_pos + 1) {
            // 如果后面还有数据，检查是否是新包头或已处理完毕
            if (data[next_pos] == 0x5A) {
                if (data[next_pos + 1] != 0xA5) {
                    // 可能是payload恰好是0x5A，但下一字节不是0xA5，说明包边界可能有问题
                    RCLCPP_WARN(logger_, "包边界疑似错误: 下一字节是0x5A但后续不是0xA5，位置=%zu", next_pos);
                }
            }
        }
        
        // 额外验证：对于设备识别包(func_code=0x01)，检查payload合理性
        if (func_code == 0x01 && packet_type == 0x06 && expected_length == 9) {
            uint8_t payload = packet[8];
            // 设备ID的合理范围：1-6 (指压01/滚轮02/负压03/冲击波04/旋转05)
            if (payload == 0x5A || payload == 0xA5 || payload > 10 || payload == 0x00) {
                RCLCPP_WARN(logger_, "设备识别包payload异常: 0x%02X, 完整包=%s, 疑似粘包错位, 跳过此包", 
                           payload, packet_hex.c_str());
                
                // 打印缓冲区剩余数据用于诊断
                if (data.size() > processed_bytes + expected_length) {
                    std::vector<uint8_t> remaining(data.begin() + processed_bytes + expected_length, 
                                                  data.begin() + std::min(processed_bytes + expected_length + 20, data.size()));
                    std::string remaining_hex = utils::bytesToHexString(remaining);
                    RCLCPP_WARN(logger_, "[粘包检测] 后续数据: %s", remaining_hex.c_str());
                }
                
                processed_bytes += 1; 
                continue;
            }
        }

        // 处理有效数据包
        std::string hex_str = utils::bytesToDatabaseHexString(packet);
        RCLCPP_DEBUG(logger_, "处理串口数据包: %s (length=%zu, type=0x%02X, func=0x%02X)",
                    hex_str.c_str(), expected_length, packet_type, func_code);

        // 通过回调函数处理数据包
        if (callback) {
            callback(hex_str);
        }

        // 更新处理位置
        processed_bytes += expected_length;
        RCLCPP_DEBUG(logger_, "处理完成，更新处理位置到: %zu", processed_bytes);
    }

    // 清除已处理的数据
    data.erase(data.begin(), data.begin() + processed_bytes);
}

} 
