// 管理器类：只负责持有/调用协议实例、将原始数据提交给协议解析、返回协议状态
// 不包含具体解析逻辑


#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include "protocol_interface.hpp"
#include <rclcpp/clock.hpp>

namespace massage_head_manager
{

class MassageHeadManager
{
public:
    explicit MassageHeadManager(rclcpp::Clock::SharedPtr clock);
    ~MassageHeadManager() = default;

    // 将收到的原始二进制包传入，返回解析结果
    BaseParseResult parseIncomingData(const std::vector<uint8_t> &data);

    // 从当前 active_protocol_ 读取当前内部状态
    InternalStatus getCurrentStatus() const;

    // 切换/设置协议实现
    void setActiveProtocol(std::shared_ptr<IProtocol> proto);

private:
    std::shared_ptr<IProtocol> active_protocol_;
    mutable std::mutex proto_mutex_;
};

}
