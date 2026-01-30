/**
 * @brief
 * 协议管理
 * 负责协调管理不同协议实现
 * 为多个协议提供统一管理接口
 * 
 */


#include "massage_head_manager/massage_head_base.hpp"
#include "massage_head_manager/protocol_v1.hpp"
#include <rclcpp/rclcpp.hpp>

namespace massage_head_manager
{

MassageHeadManager::MassageHeadManager(rclcpp::Clock::SharedPtr clock)
{
    (void)clock; 
}

BaseParseResult MassageHeadManager::parseIncomingData(const std::vector<uint8_t> &data)
{
    std::lock_guard<std::mutex> lk(proto_mutex_);
    BaseParseResult res;
    if (!active_protocol_) {
        res.success = false;
        res.raw_data = "no active protocol";
        return res;
    }
    res = active_protocol_->parsePacket(data);

    active_protocol_->printParseResult(res);
    return res;
}

InternalStatus MassageHeadManager::getCurrentStatus() const
{
    std::lock_guard<std::mutex> lk(proto_mutex_);
    if (!active_protocol_) {
        InternalStatus status;
        // 协议未初始化，返回空状态
        return status;
    }
    return active_protocol_->getCurrentStatus();
}

void MassageHeadManager::setActiveProtocol(std::shared_ptr<IProtocol> proto)
{
    std::lock_guard<std::mutex> lk(proto_mutex_);
    active_protocol_ = proto;
}

} 