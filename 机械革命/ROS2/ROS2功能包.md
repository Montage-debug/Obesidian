
#ROS2 


创建工作空间并进入src
```
mkdir test_ws/src
cd test_ws/src
```
## 创建功能包

在src中使用
```
ros2 pkg create <如：pkg_name> --build-type ament_cmake --dependencies <如：std_msgs>
```


创建完成后在功能包内的src中添加cpp文件
重点关注以下两个文件
```
CMakeLists.txt
package.xml
```

## CMakeLists
在`CMakeLists.txt`中添加以下内容：
1. 查找当前功能包所用到的包
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# 在这里添加其他查找包
```

2. 添加执行文件
```cmake
add_executable(node_test src/node_test.cpp)
```

3. 添加编译依赖
```cmake
ament_target_dependencies(
	node_test
	rclcpp
	# 在这里添加其他依赖包
)
```

4. 添加安装
```cmake
install(TARGETS
	node_test
	# 可以在这里连续添加其他节点文件
	DESTINATION lib/${PROJECT_NAME})
```


## 使用自定义头文件
当主程序文件需要使用hpp头文件来封装函数时，
`src`同目录下创建形如`头文件.cpp`和`头文件.hpp`的两个文件
以wlzc按摩机器人行为树为例：
``
在`src`目录下
`massage_bt_tree.cpp`主程序文件：
```cpp
#include "massage_bt_nodes.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BTROSNode>();
	
	// 省略具体内容...  
	
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

`massage_bt_nodes.hpp`自定义头文件：
```cpp
#ifndef MASSAGE_BT_NODES_HPP
#define MASSAGE_BT_NODES_HPP

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

// 声明类
class BTROSNode : public rclcpp::Node
{
public:
	// 声明构造函数
    BTROSNode();

private:
    
};

// 省略具体内容...  

#endif
```

`massage_bt_nodes.cpp`源文件：
```cpp
#include "massage_bt_nodes.hpp"
#include "behaviortree_cpp/behavior_tree.h"

// 实际定义
BTROSNode::BTROSNode() : Node("BT_Tree_ROS_Node")
{
	// 省略具体内容...  
}
```

最后，在`CMakeLists.txt`中修改：
```cmake
# 在这里添加可执行文件时，也需要加上源文件，这样才会编译原文件
add_executable(massage_bt_tree
  src/massage_bt_tree.cpp
  src/massage_bt_nodes.cpp)   # 关键是这个
  
ament_target_dependencies(massage_bt_tree
  rclcpp std_msgs sensor_msgs tf2_ros robot_interfaces behaviortree_cpp
)

```