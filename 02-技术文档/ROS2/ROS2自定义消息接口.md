#ROS2



服务端
1、处理请求并响应


客户端
1、等待服务
2、创建请求
3、发送请求
4、等待响应







# 一、客户端
ros2中消息接口也是一个功能包的形式，参考[[ROS2功能包]]

在功能包的基础上，创建子文件夹，用于放置自定义消息格式文件：
==topic== 消息格式：`msg\`文件夹
==service== 消息格式：`srv\`文件夹
==action== 消息格式：`action\`文件夹

## topic 消息格式

在`msg\`文件夹下创建`.msg`后缀的文件，内容如下：
```c
# 这是一个例子
string function
string content
```
## service 消息格式

在`srv\`文件夹下创建`.srv`后缀的文件，内容如下：
```c
# 请求部分
bool state
---
# 响应部分
int32 return_code
string return_msg
```
以`---`分隔请求部分和响应部分的消息格式

## action 消息格式

在`action\`文件夹下创建`.action`后缀的文件，内容如下：

```c
# Goal
geometry_msgs/Transform[] waypoints
float32[] velocity
float32[] acceleration
---
# Result
bool success
int32 processed_count
string message
---
# Feedback
geometry_msgs/Transform current_point
int32 current_index
float32 progress_percentage

```
以`---`分隔目标、结果和反馈部分的消息格式


## CMakeLists.txt 内容


```C
cmake_minimum_required(VERSION 3.8)
project(your_package_name)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/YourServiceName.srv"
)

# 编译服务端
add_executable(server_node src/server_node.cpp)
ament_target_dependencies(server_node rclcpp)

# 编译客户端
add_executable(client_node src/client_node.cpp)
ament_target_dependencies(client_node rclcpp)

# 安装
install(TARGETS
  server_node
  client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
)
```

## Package.xml
```C
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypelocation="http://download.ros.org/schema/package_format3.xsd"?>
<package format="3">
  <name>your_package_name</name>
  <version>0.0.0</version>
  <description>Package description</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```



写完后正常编译这个功能包就能导出所需的消息格式了

## 消息格式中的结构体



```c
# 
# 机械臂动作轨迹接口定义
# 
# 

# Goal
geometry_msgs/Transform[] waypoints
bool[] use_tcp_offset
float32[] velocity
float32[] acceleration
---
# Result
bool success
int32 processed_count
string message
---
# Feedback
geometry_msgs/Transform current_point
int32 current_index
float32 progress_percentage


```



# 二、服务端
