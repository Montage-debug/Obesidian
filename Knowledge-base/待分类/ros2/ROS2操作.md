
---

### 1. 禁用占用项

```vbnet
sudo systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
```

重启电脑～再次尝试

### 2. 卸载占用项

```csharp
sudo apt-get remove --purge brltty
```

重启电脑～再次尝试

---

单次生效，立即生效

```bash
sudo chmod 666 /dev/ttyUSB0
```

给当前用户添加永久权限，重启生效

```bash
sudo usermod -aG dialout `whoami`
```


---
ros2_ws/                      # 工作空间（workspace）
├── src/                      # 源代码目录
│   ├── my_robot_bringup/     # 你的功能包 (package)
│   │   ├── launch/           # 放置 launch 文件
│   │   │   ├── bringup.launch.py
│   │   │   └── simulation.launch.py
│   │   ├── config/           # 参数配置文件（YAML）
│   │   ├── urdf/             # 机器人模型文件
│   │   ├── meshes/           # 三维模型（STL/DAE）
│   │   ├── package.xml       # 包描述文件
│   │   ├── setup.py / CMakeLists.txt
│   │   └── ...
│   └── 其他功能包...
├── build/                    # colcon 编译生成的中间文件
├── install/                  # 安装后的包（launch 文件会被复制到这里）
└── log/                      # 运行日志

---


c_cpp_properties.json示例如下：

```
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/**",
                "/opt/ros/humble/include/rclcpp"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}

```


```

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("helloworld_node");
  RCLCPP_INFO(node->get_logger(),"hello world!");
  rclcpp::shutdown();
  return 0;
}
```
Python实例化Node示例如下：

```
import rclpy

def main():
    rclpy.init()
    node = rclpy.create_node("helloworld_py_node")
    node.get_logger().info("hello world!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
C++继承Node实现示例如下：

```
#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node{
public:
    MyNode():Node("node_name"){
        RCLCPP_INFO(this->get_logger(),"hello world!");
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::shutdown();
    return 0;
}

```


Python继承Node实现示例如下：

```
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("node_name_py")
        self.get_logger().info("hello world!")
def main():

    rclpy.init()
    node = MyNode() 
    rclpy.shutdown()
```


**1.根标签**


```
<package>：该标签为整个xml文件的根标签，format属性用来声明文件的格式版本。
```


2.元信息标签
```
<name> ：包名；
<version>：包的版本号；
<description>：包的描述信息；
<maintainer>：维护者信息；
<license>：软件协议；
<url>：包的介绍网址；
<author>：包的作者信息。
```




```
**3.依赖项**
- <buildtool_depend>：声明编译工具依赖；
- <build_depend>：声明编译依赖；
- <build_export_depend>：声明根据此包构建库所需依赖；
- <exec_depend>：声明执行时依赖；
- <depend>：相当于<build_depend>、<build_export_depend>、<exec_depend>三者的集成；
- <test_depend>：声明测试依赖；
- <doc_depend>：声明构建文档依赖。
```
```
##### 1.创建

新建功能包语法如下：

ros2 pkg create 包名 --build-type 构建类型 --dependencies 依赖列表 --node-name 可执行程序名称
格式解释：

- --build-type：是指功能包的构建类型，有cmake、ament_cmake、ament_python三种类型可选；
- --dependencies：所依赖的功能包列表；
- --node-name：可执行程序的名称，会自动生成对应的源文件并生成配置文件。




```


```

##### 2.编译

编译功能包语法如下：
==colcon build==
或
colcon build --packages-select 功能包列表

前者会构建工作空间下的所有功能包，后者可以构建指定功能包。

3.查找

在`ros2 pkg`命令下包含了多个查询功能包相关信息的参数。
ros2 pkg executables [包名] # 输出所有功能包或指定功能包下的可执行程序。
ros2 pkg list # 列出所有功能包
ros2 pkg prefix 包名 # 列出功能包路径
ros2 pkg xml # 输出功能包的package.xml内容

##### 4.执行

执行命令语法如下：
ros2 run 功能包 可执行程序 参数
```



### 3.自定义消息接口包
```c
1、包结构
	plaintext
	my_interfaces/
	├── msg/          # 消息文件（.msg）
	├── srv/          # 服务文件（.srv）
	├── action/       # 动作文件（.action）
	├── CMakeLists.txt  # 配置编译规则
	└── package.xml    # 配置依赖
	
==注：根据通信需求，定义`msg`（单向数据传输）、`srv`（请求 - 响应）、`action`（带反馈的长任务）。==

```

```c
1. 消息（.msg）：单向数据传递
   
int32 num # 整数 
string str # 字符串 
float64 value # 浮点数
```

```c
2. 服务（.srv）：请求 - 响应模式

int32 a         # 请求：输入a
int32 b         # 请求：输入b
---
int32 sum       # 响应：和
```

```c
3. 动作（.action）：带反馈的长任务

action
int32 order     # 目标：计算到第n项
---
int32[] sequence  # 结果：斐波那契数列
---
int32[] partial_sequence  # 反馈：中间计算结果
```

```c
**CMakelists.cpp**



cmake_minimum_required(VERSION 3.8) 
project(my_interfaces) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") add_compile_options(-Wall -Wextra -Wpedantic) 
endif() 

# 查找依赖：接口生成工具、基础消息包（如需要） 
find_package(ament_cmake REQUIRED) 
find_package(rosidl_default_generators REQUIRED) # 核心：接口生成器 
find_package(std_msgs REQUIRED) # 若依赖标准消息（如string），需添加 

# 声明要生成的接口文件（msg/srv/action） 
rosidl_generate_interfaces(${PROJECT_NAME} 
"msg/Num.msg" # 消息文件 
"srv/AddTwoInts.srv" # 服务文件 
"action/Fibonacci.action" # 动作文件 
DEPENDENCIES std_msgs # 依赖的其他消息包（如std_msgs） ) 
ament_package()

```

```python
package.xml

<?xml version="1.0"?> 
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?> 
<package format="3"> 
<name>my_interfaces</name> 
<version>0.0.0</version> 
<description>自定义接口包</description> <maintainer email="your@email.com">Your Name</maintainer> 
<license>Apache-2.0</license> 
<!-- 构建依赖：接口生成工具 --> <buildtool_depend>ament_cmake</buildtool_depend> <build_depend>rosidl_default_generators</build_depend> <build_depend>std_msgs</build_depend> <!-- 依赖的标准消息包 --> 
<!-- 运行依赖：接口运行时库 --> <exec_depend>rosidl_default_runtime</exec_depend> <exec_depend>std_msgs</exec_depend> 

<member_of_group>rosidl_interface_packages</member_of_group> <!-- 声明为接口包 --> <export> <build_type>ament_cmake</build_type> </export> </package>

```

### 4.自定义消息接口库实例  deepseek案例

#### **4.1 使用自定义消息（以发布 - 订阅为例）**
```cpp

发布者节点

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/num.hpp"

// 导入自定义消息头文件 
int main(int argc, char * argv[]) 
{
 rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("num_publisher"); // 创建发布者，主题名为"num_topic"，队列长度10 
  auto publisher = node->create_publisher<my_interfaces::msg::Num>("num_topic", 10); 
  rclcpp::Rate rate(1); // 1Hz发布频率 
  my_interfaces::msg::Num msg; // 实例化消息对象 
  int count = 0; 
  while (rclcpp::ok()) 
  { 
  msg.num = count++; 
  msg.str = "hello"; 
  msg.value = 3.14; 
  RCLCPP_INFO(node->get_logger(), "发布: num=%d, str=%s, value=%.2f", msg.num,    msg.str.c_str(), msg.value);
  publisher->publish(msg); rate.sleep(); 
  } 
  rclcpp::shutdown(); 
  return 0; 
}
```

```cpp
订阅者节点

#include "rclcpp/rclcpp.hpp" 
#include "my_interfaces/msg/num.hpp" 
void callback(const my_interfaces::msg::Num & msg) 
{ // 回调函数处理消息 
RCLCPP_INFO(rclcpp::get_logger("num_subscriber"), "接收: num=%d, str=%s, value=%.2f", msg.num, msg.str.c_str(), msg.value); 
}
int main(int argc, char * argv[]) 
{
 rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("num_subscriber"); // 创建订阅者，订阅"num_topic"，回调函数为callback 
auto subscriber = node->create_subscription<my_interfaces::msg::Num>( "num_topic", 10, callback); rclcpp::spin(node); // 循环等待消息 
rclcpp::shutdown(); 
return 0; 
}
```

```
CMakelists.txt

# 在原有基础上添加（假设节点文件在src/下） 
add_executable(publisher src/publisher_node.cpp) ament_target_dependencies(publisher rclcpp my_interfaces) # 依赖自定义接口包 install(TARGETS publisher DESTINATION lib/${PROJECT_NAME}) add_executable(subscriber src/subscriber_node.cpp) ament_target_dependencies(subscriber rclcpp my_interfaces) 
install(TARGETS subscriber DESTINATION lib/${PROJECT_NAME})
```

#### **4.2
使用自定义服务（以请求 - 响应为例）**

**场景**：服务端提供`AddTwoInts`服务（计算两数之和），客户端发送请求。

实现类似消息，核心差异是：

- 服务端：用`create_service`创建服务，定义回调函数处理请求并返回响应。
- 客户端：用`create_client`创建客户端，发送`请求对象`并等待响应。

示例（C++ 服务端核心代码）：
cpp
运行

```cpp
#include "my_interfaces/srv/add_two_ints.hpp"  // 导入服务头文件

void add(const std::shared_ptr<my_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<my_interfaces::srv::AddTwoInts::Response> response) {
  response->sum = request->a + request->b;  // 处理请求
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "收到: a=%d, b=%d, 响应: sum=%d",
             request->a, request->b, response->sum);
}

int main(...) {
  // ... 初始化节点
  auto service = node->create_service<my_interfaces::srv::AddTwoInts>("add_two_ints", &add);
  rclcpp::spin(node);
}
```

#### **4.3 使用自定义动作（以斐波那契为例）**

动作较复杂，需实现**动作服务器**（处理目标、发送反馈、返回结果）和**动作客户端**（发送目标、接收反馈和结果）。

核心类（C++）：

- 动作服务器：`rclcpp_action::Server<my_interfaces::action::Fibonacci>`
- 动作客户端：`rclcpp_action::Client<my_interfaces::action::Fibonacci>`

示例（动作服务器核心逻辑）：

cpp

运行

```cpp
#include "my_interfaces/action/fibonacci.hpp"

// 处理目标请求
void handle_goal(...) {
  // 接受目标，开始计算斐波那契数列
  auto goal = std::make_shared<my_interfaces::action::Fibonacci::Goal>();
  // ... 启动线程计算，过程中通过send_feedback发送中间结果
}

// 计算过程中发送反馈
void publish_feedback(...) {
  auto feedback = std::make_shared<my_interfaces::action::Fibonacci::Feedback>();
  feedback->partial_sequence = {1, 1, 2};  // 示例中间结果
  server->send_feedback(goal_handle, feedback);
}

// 完成后返回结果
void goal_complete(...) {
  auto result = std::make_shared<my_interfaces::action::Fibonacci::Result>();
  result->sequence = {1, 1, 2, 3, 5};  // 最终结果
  goal_handle->succeed(result);
}
```
### 4.4 #验证接口

编译节点后，可通过命令行工具验证接口是否生效：

- 查看消息类型：`ros2 interface show my_interfaces/msg/Num`
- 查看服务类型：`ros2 interface show my_interfaces/srv/AddTwoInts`
- 运行节点测试：启动发布者和订阅者，观察是否正常通信。

### **总结**

自定义接口的核心流程是：**定义接口文件→配置编译规则→生成代码→在节点中调用**。需注意：

- 接口名称避免特殊字符（仅字母、数字、下划线）。
- 修改接口后必须重新编译，并`source`环境。
- 动作接口需额外处理目标取消、超时等逻辑，比消息 / 服务更复杂。

通过自定义接口，可灵活扩展 ROS 2 的通信能力，适配各种业务场景。


#在robot_interfaces包内的CMakeLists.txt内需包括

```
robot_interfaces/CMakeLists.txt 
find_package(ament_cmake REQUIRED) 
find_package(rosidl_default_generators REQUIRED) # 消息生成工具 
# 声明要编译的消息文件（关键） 
rosidl_generate_interfaces(${PROJECT_NAME} 
"msg/massage_head_recog.msg" # 你的消息文件 # 其他消息/服务/动作文件... 
DEPENDENCIES # 如果依赖其他消息包（如std_msgs），需添加 # std_msgs 
) 

ament_package()

```

```
1. `robot_interfaces/msg/`下的文件：`MassageHeadRecog.msg`（例如不能写成`massage_head_recog.msg`）；
2. `robot_interfaces/CMakeLists.txt`中声明的文件：`"msg/MassageHeadRecog.msg"`（与实际文件名一致）；
3. 生成的头文件：`massage_head_recog.hpp`（ROS 2 会自动将`.msg`的驼峰名转为 “小写 + 下划线” 的头文件名，无需修改代码中的引用）。
```

```
#### 1. 检查 `package.xml` 关键声明

打开 `robot_interfaces/package.xml`，确保包含以下内容（声明包为 “接口包”）：

xml

```xml
<package format="3">
  <!-- 其他内容（name、version、depend 等） -->

  <!-- 必须添加：声明为 ROS 2 接口包，才会生成消息/服务代码 -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
```