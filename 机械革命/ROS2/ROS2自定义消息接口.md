#ROS2

ros2中消息接口也是一个功能包的形式，参考[[ROS2功能包]]

在功能包的基础上，创建子文件夹，用于放置自定义消息格式文件：
==topic== 消息格式：`msg\`文件夹
==service== 消息格式：`srv\`文件夹
==action== 消息格式：`action\`文件夹


```c
# 查看 /set_massage_mode 服务的实际类型
ros2 service type /set_massage_mode
```

|控制类型|推荐接口|示例|
|---|---|---|
|状态上报（周期发布）|**msg**|温度、速度、电机状态|
|即时控制（一次请求）|**srv**|模式切换、温度设定|
|任务控制（有执行过程）|**action**|执行按摩程序序列|


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


```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)  # 这个包特别重要，用于生成ros消息格式

# 声明要生成的文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RobotWaypoints.action"  # action消息格式声明
  "msg/NodeOperation.msg"   # topic消息格式声明
  "srv/SetJointPose.srv"    # service消息格式声明
  DEPENDENCIES action_msgs geometry_msgs   # 这里添加依赖功能包
)
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


