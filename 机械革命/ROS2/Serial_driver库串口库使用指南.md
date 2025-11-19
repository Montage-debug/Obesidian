```markdown
# ROS 2 serial_driver 使用指南

`serial_driver` 是 ROS 2 中用于串口通信的官方驱动包，基于 `libserial` 库封装了 ROS 2 风格的接口，提供了节点、参数配置、数据收发等功能。

## 一、核心头文件

```cpp
#include "serial_driver/serial_driver.hpp"
```

该头文件包含了 `SerialDriver` 类的定义、配置结构体（如 `SerialPortConfig`）、错误码等。

## 二、常用函数/接口

`SerialDriver` 类是 `serial_driver` 的核心，以下是其常用函数及功能：

| 函数/接口 | 功能描述 |
|-----------|----------|
| `SerialDriver(rclcpp::NodeBaseInterface::SharedPtr node)` | 构造函数，传入 ROS 2 节点的基础接口 |
| `bool init()` | 初始化驱动（读取参数、配置串口） |
| `bool open()` | 打开串口 |
| `bool close()` | 关闭串口 |
| `bool is_open() const` | 判断串口是否已打开 |
| `ssize_t send(const uint8_t *data, size_t len)` | 发送数据（字节流） |
| `ssize_t receive(uint8_t *buffer, size_t max_len, std::chrono::milliseconds timeout)` | 接收数据到缓冲区 |
| `void set_config(const SerialPortConfig &config)` | 设置串口配置 |
| `const SerialPortConfig &get_config() const` | 获取当前串口配置 |

## 三、关键数据结构

`SerialPortConfig` 用于存储串口参数：

```cpp
struct SerialPortConfig {
  std::string device_name;   // 串口设备路径（如 "/dev/ttyUSB0"）
  uint32_t baud_rate;        // 波特率（如 115200）
  uint8_t data_bits;         // 数据位（5/6/7/8，默认 8）
  std::string parity;        // 校验位（"none"/"odd"/"even"，默认 "none"）
  uint8_t stop_bits;         // 停止位（1/2，默认 1）
  std::string flow_control;  // 流控（"none"/"rtscts"，默认 "none"）
};
```

## 四、实例代码

### 1. 串口发送节点（定时发送数据）

功能：初始化串口，每秒发送一次字符串 "Hello from ROS 2 Serial!"。

```cpp
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class SerialSenderNode : public rclcpp::Node {
public:
  SerialSenderNode() : Node("serial_sender_node") {
    // 初始化 SerialDriver
    serial_driver_ = std::make_unique<serial_driver::SerialDriver>(this->get_node_base_interface());

    // 配置串口参数
    serial_driver::SerialPortConfig config;
    config.device_name = "/dev/ttyUSB0";
    config.baud_rate = 115200;
    config.data_bits = 8;
    config.parity = "none";
    config.stop_bits = 1;
    config.flow_control = "none";

    // 设置配置并初始化驱动
    serial_driver_->set_config(config);
    if (!serial_driver_->init()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial driver");
      rclcpp::shutdown();
      return;
    }

    // 打开串口
    if (!serial_driver_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", config.device_name.c_str());
      rclcpp::shutdown();
      return;
    }

    // 定时发送数据（1Hz）
    timer_ = this->create_wall_timer(1s, std::bind(&SerialSenderNode::send_data, this));
  }

  ~SerialSenderNode() {
    if (serial_driver_->is_open()) {
      serial_driver_->close();
    }
  }

private:
  void send_data() {
    std::string msg = "Hello from ROS 2 Serial!\r\n";
    const uint8_t *data = reinterpret_cast<const uint8_t*>(msg.data());
    ssize_t sent = serial_driver_->send(data, msg.size());

    if (sent == msg.size()) {
      RCLCPP_INFO(this->get_logger(), "Sent: %s", msg.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to send data");
    }
  }

  std::unique_ptr<serial_driver::SerialDriver> serial_driver_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialSenderNode>());
  rclcpp::shutdown();
  return 0;
}
```

### 2. 串口接收节点（循环接收数据并打印）

功能：初始化串口，循环接收数据并打印到控制台。

```cpp
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <vector>
#include <thread>

class SerialReceiverNode : public rclcpp::Node {
public:
  SerialReceiverNode() : Node("serial_receiver_node") {
    // 初始化 SerialDriver
    serial_driver_ = std::make_unique<serial_driver::SerialDriver>(this->get_node_base_interface());

    // 配置串口参数
    serial_driver::SerialPortConfig config;
    config.device_name = "/dev/ttyUSB0";
    config.baud_rate = 115200;
    config.data_bits = 8;
    config.parity = "none";
    config.stop_bits = 1;
    config.flow_control = "none";

    // 初始化并打开串口
    serial_driver_->set_config(config);
    if (!serial_driver_->init() || !serial_driver_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize or open serial port");
      rclcpp::shutdown();
      return;
    }

    // 创建接收线程
    receive_thread_ = std::thread(&SerialReceiverNode::receive_data, this);
  }

  ~SerialReceiverNode() {
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (serial_driver_->is_open()) {
      serial_driver_->close();
    }
  }

private:
  void receive_data() {
    const size_t buffer_size = 1024;
    std::vector<uint8_t> buffer(buffer_size);

    while (rclcpp::ok()) {
      // 接收数据（超时 100ms）
      ssize_t received = serial_driver_->receive(buffer.data(), buffer_size, std::chrono::milliseconds(100));

      if (received > 0) {
        std::string data_str(buffer.begin(), buffer.begin() + received);
        RCLCPP_INFO(this->get_logger(), "Received %zd bytes: %s", received, data_str.c_str());
      } else if (received < 0) {
        RCLCPP_WARN(this->get_logger(), "Receive error");
      }
    }
  }

  std::unique_ptr<serial_driver::SerialDriver> serial_driver_;
  std::thread receive_thread_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
```

### 3. 收发一体节点（收到数据后回复）

功能：接收数据后，立即回复 "Received: [收到的数据]"。

```cpp
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <vector>
#include <thread>

class SerialEchoNode : public rclcpp::Node {
public:
  SerialEchoNode() : Node("serial_echo_node") {
    serial_driver_ = std::make_unique<serial_driver::SerialDriver>(this->get_node_base_interface());

    // 通过 ROS 2 参数动态设置串口参数
    this->declare_parameter("device_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("data_bits", 8);
    this->declare_parameter("parity", "none");
    this->declare_parameter("stop_bits", 1);
    this->declare_parameter("flow_control", "none");

    serial_driver::SerialPortConfig config;
    this->get_parameter("device_name", config.device_name);
    this->get_parameter("baud_rate", config.baud_rate);
    this->get_parameter("data_bits", config.data_bits);
    this->get_parameter("parity", config.parity);
    this->get_parameter("stop_bits", config.stop_bits);
    this->get_parameter("flow_control", config.flow_control);

    // 初始化并打开串口
    serial_driver_->set_config(config);
    if (!serial_driver_->init() || !serial_driver_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", config.device_name.c_str());
      rclcpp::shutdown();
      return;
    }

    // 启动收发线程
    echo_thread_ = std::thread(&SerialEchoNode::echo_data, this);
  }

  ~SerialEchoNode() {
    if (echo_thread_.joinable()) {
      echo_thread_.join();
    }
    if (serial_driver_->is_open()) {
      serial_driver_->close();
    }
  }

private:
  void echo_data() {
    const size_t buffer_size = 512;
    std::vector<uint8_t> buffer(buffer_size);

    while (rclcpp::ok()) {
      // 接收数据（超时 500ms）
      ssize_t received = serial_driver_->receive(buffer.data(), buffer_size, std::chrono::milliseconds(500));

      if (received > 0) {
        std::string recv_str(buffer.begin(), buffer.begin() + received);
        RCLCPP_INFO(this->get_logger(), "Received: %s", recv_str.c_str());

        // 构造回复数据
        std::string reply = "Received: " + recv_str + "\r\n";
        ssize_t sent = serial_driver_->send(reinterpret_cast<const uint8_t*>(reply.data()), reply.size());

        if (sent != reply.size()) {
          RCLCPP_WARN(this->get_logger(), "Failed to send reply");
        }
      }
    }
  }

  std::unique_ptr<serial_driver::SerialDriver> serial_driver_;
  std::thread echo_thread_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialEchoNode>());
  rclcpp::shutdown();
  return 0;
}
```

## 五、编译配置

在 `package.xml` 中添加依赖：

```xml
<depend>serial_driver</depend>
<depend>rclcpp</depend>
```

在 `CMakeLists.txt` 中添加：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)

add_executable(serial_sender src/serial_sender.cpp)
ament_target_dependencies(serial_sender rclcpp serial_driver)
install(TARGETS serial_sender DESTINATION lib/${PROJECT_NAME})

# 同理添加其他节点的编译配置
```

## 总结

- `serial_driver` 的核心是 `SerialDriver` 类，通过它实现串口的初始化、配置、读写
- 实际使用时需注意串口参数（波特率、设备路径等）与外设匹配
- 发送/接收数据需以字节流形式处理
- 接收操作建议用线程或回调避免阻塞 ROS 2 节点的主循环
```