
/**
 * @brief
 * 提供数据格式转换工具
 * 
 */


#pragma once  //防止头文件重复包含
// 工具：bytes <-> hex string，以及小工具
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <ctime>
#include "rclcpp/rclcpp.hpp"

namespace massage_head_manager {
namespace utils {

// bytes -> "AA BB CC"（大写 hex，空格分隔）
inline std::string bytesToHexString(const std::vector<uint8_t>& bytes) {
    std::ostringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < bytes.size(); ++i) {
        ss << std::setw(2) << static_cast<int>(bytes[i]);
        if (i + 1 < bytes.size()) ss << " ";
    }
    return ss.str();
}

// 支持 "AABBCC" 或 "AA BB CC"
inline std::vector<uint8_t> hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> out;
    // 简化：先构造只包含 hex chars 的字符串
    std::string filtered;
    filtered.reserve(hex.size());
    for (char c : hex) {
        if (std::isxdigit(static_cast<unsigned char>(c))) filtered.push_back(c);
    }
    // 如果长度为奇数，前面补 0
    if (filtered.size() % 2 == 1) filtered.insert(filtered.begin(), '0');
    for (size_t i = 0; i + 1 < filtered.size(); i += 2) {
        std::string byteStr = filtered.substr(i, 2);
        uint8_t b = static_cast<uint8_t>(std::strtoul(byteStr.c_str(), nullptr, 16));
        out.push_back(b);
    }
    return out;
}

// bytes -> "AA BB CC DD EE"
// 格式：5A A5 06 83 10 01 01 00 05
// 与数据库中的 5A-A5-06-83-10-01-01-00-05 通过 hexHyphensToSpaces 转换后匹配
inline std::string bytesToDatabaseHexString(const std::vector<uint8_t>& bytes) {
    // 使用标准格式，所有字节都用空格分隔
    return bytesToHexString(bytes);
}

/**
 * @brief 将ROS2时间转换为ISO 8601格式字符串
 * @param time ROS2时间对象
 * @return ISO 8601格式时间字符串 (例: "2024-12-22T03:45:30.123Z")
 * @details 生成符合ISO 8601标准的UTC时间戳，包含毫秒精度
 */
inline std::string toIso8601String(const rclcpp::Time& time) {
    // 获取秒和纳秒部分
    auto seconds = time.seconds();
    auto nanoseconds = time.nanoseconds();

    // 转换为time_t用于格式化
    std::time_t time_t_val = static_cast<std::time_t>(seconds);
    std::tm* tm_info = std::gmtime(&time_t_val);

    // 格式化为 ISO 8601 基础格式
    std::ostringstream oss;
    oss << std::put_time(tm_info, "%Y-%m-%dT%H:%M:%S");

    // 添加毫秒部分
    uint32_t milliseconds = nanoseconds / 1000000;
    oss << "." << std::setfill('0') << std::setw(3)
        << (milliseconds % 1000) << "Z";

    return oss.str();
}

/**
 * @brief 将空格分隔的十六进制字符串转换为连字符分隔格式
 * @param hex_string 输入的十六进制字符串，如 "5A A5 06 83 1001 01 00 05"
 * @return 转换后的字符串，如 "5A-A5-06-83-10-01-01-00-05"
 * @details 将输入字符串中的所有空格替换为连字符，适用于串口数据的格式转换
 */
inline std::string hexSpacesToHyphens(const std::string& hex_string) {
    std::string result = hex_string;
    for (char& c : result) {
        if (c == ' ') {
            c = '-';
        }
    }
    return result;
}

/**
 * @brief 将连字符分隔的十六进制字符串转换为空格分隔格式
 * @param hex_string 输入的十六进制字符串，如 "5A-A5-06-83-10-01-01-00-05"
 * @return 转换后的字符串，如 "5A A5 06 83 1001 01 00 05"
 * @details 将输入字符串中的所有连字符替换为空格，适用于串口数据的格式转换
 */
inline std::string hexHyphensToSpaces(const std::string& hex_string) {
    std::string result = hex_string;
    for (char& c : result) {
        if (c == '-') {
            c = ' ';
        }
    }
    return result;
}


} // namespace utils
} // namespace massage_head_manager
