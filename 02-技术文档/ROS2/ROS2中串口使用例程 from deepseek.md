
#ROS2 #串口

在ROS2中使用`serial_driver`进行串口通信，以下是详细的使用方法：

## 1. 安装依赖

```bash
sudo apt-get install ros-<distro>-serial-driver
# 例如对于Humble：
sudo apt-get install ros-humble-serial-driver
```

## 2. 基本使用方法

### 创建节点并初始化serial_driver

```cpp
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        // 初始化IO上下文
        io_context_ = std::make_shared<drivers::common::IoContext>();
        
        // 配置串口参数
        auto port_name = "/dev/ttyUSB0";  // 串口设备
        auto fc = serial_driver::FlowControl::NONE;  // 流控制
        auto pt = serial_driver::Parity::NONE;       // 校验位
        auto sb = serial_driver::StopBits::ONE;      // 停止位
        
        serial_driver::SerialPortConfig config(
            115200,  // 波特率
            fc, pt, sb
        );
        
        // 创建串口驱动
        serial_driver_ = std::make_unique<serial_driver::SerialDriver>(
            *io_context_, port_name, config
        );
        
        // 启动接收线程
        receive_thread_ = std::thread(&SerialNode::receiveData, this);
        
        // 创建定时器发送数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialNode::timerCallback, this)
        );
    }
    
    ~SerialNode()
    {
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void receiveData()
    {
        std::vector<uint8_t> buffer(1024);
        
        while (rclcpp::ok()) {
            try {
                // 读取串口数据
                auto bytes_read = serial_driver_->port().receive(buffer);
                
                if (bytes_read > 0) {
                    // 处理接收到的数据
                    processReceivedData(buffer.data(), bytes_read);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "接收数据错误: %s", e.what());
            }
        }
    }
    
    void processReceivedData(const uint8_t* data, size_t length)
    {
        // 处理接收到的数据
        std::string received_str(reinterpret_cast<const char*>(data), length);
        RCLCPP_INFO(this->get_logger(), "收到数据: %s", received_str.c_str());
    }
    
    void timerCallback()
    {
        // 发送数据示例
        std::string message = "Hello Serial!\r\n";
        try {
            serial_driver_->port().send(
                reinterpret_cast<const uint8_t*>(message.data()),
                message.size()
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送数据错误: %s", e.what());
        }
    }
    
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<serial_driver::SerialDriver> serial_driver_;
    std::thread receive_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## 3. 完整的串口通信节点示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/string.hpp>

class RobotSerialDriver : public rclcpp::Node
{
public:
    RobotSerialDriver() : Node("robot_serial_driver")
    {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        
        // 获取参数
        auto port = this->get_parameter("port").as_string();
        auto baud_rate = this->get_parameter("baud_rate").as_int();
        
        // 初始化串口
        initSerial(port, baud_rate);
        
        // 创建发布者和订阅者
        received_pub_ = this->create_publisher<std_msgs::msg::String>("serial_received", 10);
        send_sub_ = this->create_subscription<std_msgs::msg::String>(
            "serial_send", 10,
            std::bind(&RobotSerialDriver::sendCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "串口驱动节点已启动，端口: %s, 波特率: %d", 
                   port.c_str(), baud_rate);
    }

private:
    void initSerial(const std::string& port, int baud_rate)
    {
        try {
            io_context_ = std::make_shared<drivers::common::IoContext>();
            
            serial_driver::SerialPortConfig config(
                baud_rate,
                serial_driver::FlowControl::NONE,
                serial_driver::Parity::NONE,
                serial_driver::StopBits::ONE
            );
            
            serial_driver_ = std::make_unique<serial_driver::SerialDriver>(
                *io_context_, port, config
            );
            
            // 启动接收线程
            receive_thread_ = std::thread(&RobotSerialDriver::receiveLoop, this);
            
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "串口初始化失败: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    void receiveLoop()
    {
        std::vector<uint8_t> buffer(1024);
        
        while (rclcpp::ok()) {
            try {
                auto bytes_read = serial_driver_->port().receive(buffer);
                
                if (bytes_read > 0) {
                    // 发布接收到的数据
                    auto message = std_msgs::msg::String();
                    message.data = std::string(
                        reinterpret_cast<const char*>(buffer.data()),
                        bytes_read
                    );
                    
                    received_pub_->publish(message);
                    
                    RCLCPP_DEBUG(this->get_logger(), "收到 %zu 字节数据", bytes_read);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "接收数据异常: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    void sendCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            serial_driver_->port().send(
                reinterpret_cast<const uint8_t*>(msg->data.c_str()),
                msg->data.size()
            );
            
            RCLCPP_DEBUG(this->get_logger(), "发送 %zu 字节数据", msg->data.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送数据失败: %s", e.what());
        }
    }
    
    // 成员变量
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<serial_driver::SerialDriver> serial_driver_;
    std::thread receive_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr received_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr send_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSerialDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 4. 参数配置文件

创建`config/serial_params.yaml`：

```yaml
robot_serial_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baud_rate: 115200
```

## 5. 启动文件

创建`launch/serial_driver.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='serial_driver_node',
            name='robot_serial_driver',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud_rate': 115200
            }]
        )
    ])
```

## 6. 使用示例

```bash
# 启动节点
ros2 launch your_package serial_driver.launch.py

# 手动发送数据
ros2 topic pub /serial_send std_msgs/msg/String "{data: 'Hello World'}"

# 查看接收到的数据
ros2 topic echo /serial_received
```

## 注意事项

1. **权限问题**：确保用户有访问串口设备的权限
   ```bash
   sudo usermod -a -G dialout $USER
   ```

2. **异常处理**：串口通信容易出错，需要完善的异常处理

3. **线程安全**：多线程访问串口时要注意同步

4. **数据解析**：根据具体协议实现数据解析逻辑

这种使用方法适合大多数串口通信场景，你可以根据具体需求进行修改和扩展。