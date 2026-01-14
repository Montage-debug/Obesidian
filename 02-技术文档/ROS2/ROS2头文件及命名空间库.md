# 一、串口相关

## 1 头文件目录

```

#include <cstdio> // C标准输入输出库（基础IO操作） 
#include <rclcpp/rclcpp.hpp> // ROS 2核心库（节点、日志、发布订阅等核心功能）
#include <std_msgs/msg/string.hpp> // ROS字符串消息类型（用于发布串口数据） 
#include <iostream> // C++标准输入输出流（控制台打印） 
#include <serial_driver/serial_driver.hpp> // ROS 2串口驱动库（处理串口通信） #include <thread> // C++线程库（创建独立线程读取串口，避免阻塞节点） 
#include <memory> // C++智能指针库（管理动态内存，如std::shared_ptr）
```

```
## 2.命名空间
using namespace std::chrono_literals; // 允许使用时间字面量（如10ms、500ms） 
using namespace drivers::serial_driver; 
// 简化串口驱动类的调用（如SerialDriver可直接使用，无需写全称drivers::serial_driver::SerialDriver）
```

# 二、hpp文件和.cpp文件设置
## 1、cpp文件配置
```c++
引用头文件：路径为 "包名/头文件名.hpp"
#include "my_package/my_class.hpp" 
可根据需要引用其他库（如ROS2的rclcpp） 
#include "rclcpp/rclcpp.hpp" 
// 在对应命名空间中实现函数 
namespace my_package 
{
 // 实现普通函数
 void print_hello() 
 { 
 RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Hello from my_package!"); 
 } 
 // 实现类的成员函数 
 void MyClass::add(int a, int b)
  { 
  RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Sum: %d", a + b);
  } 
} // namespace my_package
```

## 2、hpp文件配置
```
// 包含守卫：格式为 <包名_文件名_HPP_> 
#ifndef MY_PACKAGE_MY_CLASS_HPP_ 
#define MY_PACKAGE_MY_CLASS_HPP_ 
// 命名空间：建议与包名一致 
namespace my_package { // 声明一个普通函数 
void print_hello(); // 声明一个类及成员函数 
class MyClass { public: // 成员函数声明
void add(int a, int b); }; } // namespace my_package 
#endif // MY_PACKAGE_MY_CLASS_HPP_

```
## 3、CMakeLIst.txt配置文件
```c
cmake_minimum_required(VERSION 3.8) 
project(my_package) 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") add_compile_options(-Wall -Wextra -Wpedantic) 
endif() 


# 查找依赖（如rclcpp） 
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 

# 配置头文件目录（让编译器知道include路径） 
include_directories( include ) 

# 示例：将src中的cpp编译为库（供其他节点调用） 
add_library(my_library SHARED src/my_class.cpp ) 

# 声明库的头文件路径（必须配置，否则其他文件找不到头文件） 
target_include_directories(my_library PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 编译时路径 $<INSTALL_INTERFACE:include> # 安装后路径 ) 

# 链接依赖（如rclcpp）
ament_target_dependencies(my_library rclcpp ) 

# 安装库和头文件（可选，根据需求） 
install(TARGETS my_library 
EXPORT export_${PROJECT_NAME} 
ARCHIVE DESTINATION lib 
LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin 
 ) 
 
 install(DIRECTORY include/
  DESTINATION include 
 ) 
 
ament_export_include_directories(include)
ament_export_libraries(my_library) 
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

 
ament_package()
```
## 总结

1. **头文件路径**：`include/包名/xxx.hpp`，声明时用包含守卫和命名空间；
2. **源文件引用**：`#include "包名/xxx.hpp"`；
3. **CMake 配置**：必须通过`include_directories`和`target_include_directories`指定`include`目录，否则编译会报错 “找不到头文件”