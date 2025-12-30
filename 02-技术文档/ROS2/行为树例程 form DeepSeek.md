我将为您提供一个使用ROS2和BehaviorTree.CPP控制海龟的简单行为树例程。这个例子将创建一个行为树，让海龟执行一系列动作。

## 1. 创建ROS2包

首先创建一个新的ROS2包：

```bash
ros2 pkg create --build-type ament_cmake turtle_behavior_tree --dependencies rclcpp behaviortree_cpp geometry_msgs turtlesim
```

## 2. 创建行为树节点

创建 `include/turtle_behavior_tree/bt_nodes.hpp`：

```cpp
#ifndef TURTLE_BEHAVIOR_TREE_BT_NODES_HPP
#define TURTLE_BEHAVIOR_TREE_BT_NODES_HPP

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleBTNodes : public rclcpp::Node
{
public:
  TurtleBTNodes();
  
  // Behavior Tree 节点
  BT::NodeStatus moveForward();
  BT::NodeStatus rotate();
  BT::NodeStatus checkPosition();
  BT::NodeStatus stopTurtle();
  
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  turtlesim::msg::Pose getCurrentPose() const { return current_pose_; }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  turtlesim::msg::Pose current_pose_;
};

// 移动前进的动作节点
class MoveForwardAction : public BT::StatefulActionNode
{
public:
  MoveForwardAction(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<TurtleBTNodes> turtle_node);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<TurtleBTNodes> turtle_node_;
  rclcpp::Time start_time_;
};

// 旋转的动作节点
class RotateAction : public BT::StatefulActionNode
{
public:
  RotateAction(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<TurtleBTNodes> turtle_node);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<TurtleBTNodes> turtle_node_;
  rclcpp::Time start_time_;
  double target_angle_;
};

// 检查位置的条件节点
class CheckPositionCondition : public BT::ConditionNode
{
public:
  CheckPositionCondition(const std::string& name, const BT::NodeConfig& config,
                        std::shared_ptr<TurtleBTNodes> turtle_node);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TurtleBTNodes> turtle_node_;
};

#endif
```

## 3. 实现行为树节点

创建 `src/bt_nodes.cpp`：

```cpp
#include "turtle_behavior_tree/bt_nodes.hpp"
#include "behaviortree_cpp/behavior_tree.h"

using namespace std::chrono_literals;

TurtleBTNodes::TurtleBTNodes() : Node("turtle_bt_nodes")
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
    "/turtle1/pose", 10,
    std::bind(&TurtleBTNodes::poseCallback, this, std::placeholders::_1));
}

void TurtleBTNodes::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  current_pose_ = *msg;
}

// MoveForwardAction 实现
MoveForwardAction::MoveForwardAction(const std::string& name, const BT::NodeConfig& config,
                                   std::shared_ptr<TurtleBTNodes> turtle_node)
  : BT::StatefulActionNode(name, config), turtle_node_(turtle_node)
{
}

BT::PortsList MoveForwardAction::providedPorts()
{
  return { BT::InputPort<double>("duration", 2.0, "移动持续时间(秒)") };
}

BT::NodeStatus MoveForwardAction::onStart()
{
  double duration;
  if (!getInput("duration", duration)) {
    duration = 2.0;
  }
  
  start_time_ = turtle_node_->now();
  
  // 发布前进命令
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 2.0;
  twist.angular.z = 0.0;
  turtle_node_->cmd_vel_pub_->publish(twist);
  
  RCLCPP_INFO(turtle_node_->get_logger(), "开始前进，持续时间: %.1f秒", duration);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveForwardAction::onRunning()
{
  double duration;
  getInput("duration", duration);
  
  auto elapsed = (turtle_node_->now() - start_time_).seconds();
  if (elapsed < duration) {
    return BT::NodeStatus::RUNNING;
  }
  
  // 停止海龟
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.0;
  turtle_node_->cmd_vel_pub_->publish(twist);
  
  RCLCPP_INFO(turtle_node_->get_logger(), "前进完成");
  return BT::NodeStatus::SUCCESS;
}

void MoveForwardAction::onHalted()
{
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.0;
  turtle_node_->cmd_vel_pub_->publish(twist);
  RCLCPP_INFO(turtle_node_->get_logger(), "前进动作被中断");
}

// RotateAction 实现
RotateAction::RotateAction(const std::string& name, const BT::NodeConfig& config,
                         std::shared_ptr<TurtleBTNodes> turtle_node)
  : BT::StatefulActionNode(name, config), turtle_node_(turtle_node)
{
}

BT::PortsList RotateAction::providedPorts()
{
  return { BT::InputPort<double>("angle", 90.0, "旋转角度(度)") };
}

BT::NodeStatus RotateAction::onStart()
{
  double angle_degrees;
  if (!getInput("angle", angle_degrees)) {
    angle_degrees = 90.0;
  }
  
  start_time_ = turtle_node_->now();
  target_angle_ = angle_degrees * M_PI / 180.0; // 转换为弧度
  
  // 发布旋转命令
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.0;
  twist.angular.z = 1.0; // 固定角速度
  turtle_node_->cmd_vel_pub_->publish(twist);
  
  RCLCPP_INFO(turtle_node_->get_logger(), "开始旋转，角度: %.1f度", angle_degrees);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateAction::onRunning()
{
  // 简单的基于时间的旋转控制
  // 实际应用中应该基于海龟的实际朝向
  auto elapsed = (turtle_node_->now() - start_time_).seconds();
  double expected_rotation = elapsed * 1.0; // 角速度 1 rad/s
  
  if (expected_rotation < std::abs(target_angle_)) {
    return BT::NodeStatus::RUNNING;
  }
  
  // 停止旋转
  auto twist = geometry_msgs::msg::Twist();
  twist.angular.z = 0.0;
  turtle_node_->cmd_vel_pub_->publish(twist);
  
  RCLCPP_INFO(turtle_node_->get_logger(), "旋转完成");
  return BT::NodeStatus::SUCCESS;
}

void RotateAction::onHalted()
{
  auto twist = geometry_msgs::msg::Twist();
  twist.angular.z = 0.0;
  turtle_node_->cmd_vel_pub_->publish(twist);
  RCLCPP_INFO(turtle_node_->get_logger(), "旋转动作被中断");
}

// CheckPositionCondition 实现
CheckPositionCondition::CheckPositionCondition(const std::string& name, const BT::NodeConfig& config,
                                             std::shared_ptr<TurtleBTNodes> turtle_node)
  : BT::ConditionNode(name, config), turtle_node_(turtle_node)
{
}

BT::PortsList CheckPositionCondition::providedPorts()
{
  return { BT::InputPort<double>("x_threshold", 5.0, "X坐标阈值") };
}

BT::NodeStatus CheckPositionCondition::tick()
{
  double x_threshold;
  if (!getInput("x_threshold", x_threshold)) {
    x_threshold = 5.0;
  }
  
  auto pose = turtle_node_->getCurrentPose();
  RCLCPP_INFO(turtle_node_->get_logger(), "当前位置: x=%.2f, y=%.2f", pose.x, pose.y);
  
  if (pose.x > x_threshold) {
    RCLCPP_INFO(turtle_node_->get_logger(), "位置检查: 超过阈值");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(turtle_node_->get_logger(), "位置检查: 未超过阈值");
    return BT::NodeStatus::FAILURE;
  }
}
```

## 4. 创建主行为树程序

创建 `src/turtle_behavior_tree.cpp`：

```cpp
#include "turtle_behavior_tree/bt_nodes.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建ROS2节点
  auto turtle_node = std::make_shared<TurtleBTNodes>();
  
  // 创建行为树工厂
  BT::BehaviorTreeFactory factory;
  
  // 注册自定义节点
  factory.registerNodeType<MoveForwardAction>("MoveForward", turtle_node);
  factory.registerNodeType<RotateAction>("Rotate", turtle_node);
  factory.registerNodeType<CheckPositionCondition>("CheckPosition", turtle_node);
  
  // 创建行为树（使用XML定义）
  const std::string xml_tree = R"(
    <root BTCPP_format="4">
      <BehaviorTree>
        <Sequence>
          <MoveForward duration="1.5"/>
          <Rotate angle="90.0"/>
          <MoveForward duration="1.0"/>
          <Rotate angle="90.0"/>
          <MoveForward duration="1.0"/>
          <CheckPosition x_threshold="7.0"/>
          <Fallback>
            <CheckPosition x_threshold="7.0"/>
            <Sequence>
              <Rotate angle="180.0"/>
              <MoveForward duration="2.0"/>
            </Sequence>
          </Fallback>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
  
  auto tree = factory.createTreeFromText(xml_tree);
  
  // 创建日志记录器
  BT::StdCoutLogger logger_cout(tree);
  BT::FileLogger logger_file(tree, "turtle_bt_trace.fbl");
  
  RCLCPP_INFO(turtle_node->get_logger(), "开始执行行为树");
  
  // 执行行为树
  while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickOnce();
    
    if (status == BT::NodeStatus::RUNNING) {
      rclcpp::spin_some(turtle_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
      RCLCPP_INFO(turtle_node->get_logger(), "行为树执行完成");
      break;
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
```

## 5. 修改CMakeLists.txt

更新 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(turtle_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# 包含头文件目录
include_directories(include)

# 创建行为树节点库
add_library(bt_nodes src/bt_nodes.cpp)
ament_target_dependencies(bt_nodes 
  rclcpp geometry_msgs turtlesim behaviortree_cpp)

# 创建主可执行文件
add_executable(turtle_behavior_tree src/turtle_behavior_tree.cpp)
target_link_libraries(turtle_behavior_tree bt_nodes)
ament_target_dependencies(turtle_behavior_tree 
  rclcpp geometry_msgs turtlesim behaviortree_cpp)

# 安装
install(TARGETS
  turtle_behavior_tree
  bt_nodes
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(bt_nodes)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  turtlesim
  behaviortree_cpp)

ament_package()
```

## 6. 修改package.xml

确保 `package.xml` 包含所有依赖：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematype="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtle_behavior_tree</name>
  <version>0.0.0</version>
  <description>TurtleSim behavior tree example</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>
  <depend>behaviortree_cpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 7. 编译和运行

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_behavior_tree
source install/setup.bash

# 启动 turtlesim
ros2 run turtlesim turtlesim_node

# 运行行为树
ros2 run turtle_behavior_tree turtle_behavior_tree
```

## 行为树说明

这个行为树定义了以下流程：
1. 前进1.5秒
2. 旋转90度
3. 前进1秒
4. 旋转90度
5. 前进1秒
6. 检查位置是否超过阈值
7. 如果位置未超过阈值，则旋转180度并后退

这个例子展示了行为树的基本概念：序列(Sequence)、回退(Fallback)、条件节点和动作节点。您可以根据需要修改XML来创建更复杂的行为。