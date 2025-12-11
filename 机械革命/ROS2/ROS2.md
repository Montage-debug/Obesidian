# 一、 ROS2 中ros2 bag 命令工具
## 1、可以方便的实现数据的录制回放等操作，ros2 bag 的基本使用语法如下：

```
convert  给定一个 bag 文件，写出一个新的具有不同配置的 bag 文件；
info     输出 bag 文件的相关信息；
list     输出可用的插件信息；
play     回放 bag 文件数据；
record   录制 bag 文件数据；
reindex  重建 bag 的元数据文件。
```
## 2、 rosbag2 编程（C++）

#### 1.序列化

功能包 cpp02_rosbag 的 src 目录下，新建 C++ 文件 demo01_writer.cpp，并编辑文件，输入如下内容：

```
/* 
  需求：录制 turtle_teleop_key 节点发布的速度指令。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建写出对象指针；
      3-2.设置写出的目标文件；
      3-3.写出消息。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

 */
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

// 3.定义节点类；
class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    // 3-1.创建写出对象指针；
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    // 3-2.设置写出的目标文件；
    writer_->open("my_bag");
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();
    // 3-3.写出消息。
    writer_->write(msg, "/turtle1/cmd_vel", "geometry_msgs/msg/Twist", time_stamp);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  // 2.初始化 ROS 客户端；
  rclcpp::init(argc, argv);
  // 4.调用 spin 函数，并传入对象指针；
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  // 5.释放资源。
  rclcpp::shutdown();
  return 0;
}
```

#### 2.反序列化

功能包 cpp02_rosbag 的 src 目录下，新建 C++ 文件 demo02_reader.cpp，并编辑文件，输入如下内容：

```
/* 
  需求：读取 bag 文件数据。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建读取对象指针；
      3-2.设置读取的目标文件；
      3-3.读消息；
      3-4.关闭文件。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

 */
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "geometry_msgs/msg/twist.hpp"
// 3.定义节点类；
class SimpleBagPlayer: public rclcpp::Node {
public:
    SimpleBagPlayer():Node("simple_bag_player"){
        // 3-1.创建读取对象指针；
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        // 3-2.设置读取的目标文件；
        reader_->open("my_bag");
        // 3-3.读消息；
        while (reader_->has_next())
        {
            geometry_msgs::msg::Twist twist = reader_->read_next<geometry_msgs::msg::Twist>();
            RCLCPP_INFO(this->get_logger(),"%.2f ---- %.2f",twist.linear.x, twist.angular.z);
        }


        // 3-4.关闭文件。
        reader_->close();
    }
private:
    std::unique_ptr<rosbag2_cpp::Reader> reader_;

};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS 客户端；
    rclcpp::init(argc,argv);
    // 4.调用 spin 函数，并传入对象指针；
    rclcpp::spin(std::make_shared<SimpleBagPlayer>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
```

#### 3.编辑配置文件

##### 1.package.xml

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclcpp</depend>
<depend>rosbag2_cpp</depend>
<depend>geometry_msgs</depend>
```

##### 2.CMakeLists.txt

CMakeLists.txt 中的相关配置如下：

```
add_executable(demo01_writer src/demo01_writer.cpp)
ament_target_dependencies(
  demo01_writer
  "rclcpp"
  "rosbag2_cpp"
  "geometry_msgs"
)

add_executable(demo02_reader src/demo02_reader.cpp)
ament_target_dependencies(
  demo02_reader
  "rclcpp"
  "rosbag2_cpp"
  "geometry_msgs"
)

install(TARGETS 
  demo01_writer
  demo02_reader
  DESTINATION lib/${PROJECT_NAME})
```

#### 4.编译

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select cpp02_rosbag
```

#### 5.执行

当前工作空间下，启动两个终端，终端1执行录制程序，终端2执行回放程序。

终端1输入如下指令：

```
. install/setup.bash
ros2 run cpp02_rosbag demo01_writer
```

执行完毕后，会在当前工作空间下生成一个名为 my_bag 的目录。

终端2输入如下指令：

```
. install/setup.bash 
ros2 run cpp02_rosbag demo02_reader
```

该程序运行会读取 my_bag 中记录的数据，其结果是在终端打印录制的速度指令最终的线速度和角速度。

# 二、节点设置

launch 中需要执行的节点被封装为了 launch_ros.actions.Node 对象。

**需求：**launch 文件中配置节点的相关属性。

**示例：**

在 cpp01_launch/launch/py 目录下新建 py01_node.launch.py 文件，输入如下内容：

```
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    turtle1 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                namespace="ns_1",
                name="t1", 
                exec_name="turtle_label", # 表示流程的标签
                respawn=True)
    turtle2 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                name="t2",
                # 参数设置方式1
                # parameters=[{"background_r": 0,"background_g": 0,"background_b": 0}],
                # 参数设置方式2: 从 yaml 文件加载参数，yaml 文件所属目录需要在配置文件中安装。
                parameters=[os.path.join(get_package_share_directory("cpp01_launch"),"config","t2.yaml")],
                )
    turtle3 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                name="t3", 
                remappings=[("/turtle1/cmd_vel","/cmd_vel")] #话题重映射
                )
    rviz = Node(package="rviz2",
                executable="rviz2",
                # 节点启动时传参
                arguments=["-d", os.path.join(get_package_share_directory("cpp01_launch"),"config","my.rviz")]
    )

    turtle4 = Node(package="turtlesim", 
                executable="turtlesim_node",
                # 节点启动时传参，相当于 arguments 传参时添加前缀 --ros-args 
                ros_arguments=["--remap", "__ns:=/t4_ns", "--remap", "__node:=t4"]
    )
    return LaunchDescription([turtle1, turtle2, turtle3, rviz, turtle4])
```

**代码解释：**

**1.Node使用语法1**

```
turtle1 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                namespace="group_1", 
                name="t1", 
                exec_name="turtle_label", # 表示流程的标签
                respawn=True)
```

上述代码会创建一个 turtlesim_node 节点，设置了若干节点属性，并且节点关闭后会自动重启。

- package：功能包；
- executable：可执行文件；
- namespace：命名空间；
- name：节点名称；
- exe_name：流程标签；
- respawn：设置为True时，关闭节点后，可以自动重启。

**2.Node使用语法2**

```
turtle2 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                name="t2",
                # 参数设置方式1
                # parameters=[{"background_r": 0,"background_g": 0,"background_b": 0}],
                # 参数设置方式2: 从 yaml 文件加载参数，yaml 文件所属目录需要在配置文件中安装。
                parameters=[os.path.join(get_package_share_directory("cpp01_launch"),"config","t2.yaml")],
                )
```

上述代码会创建一个 turtlesim_node 节点，并导入背景色相关参数。

- parameters：导入参数。

parameter 用于设置被导入的参数，如果是从 yaml 文件加载参数，那么需要先准备 yaml 文件，在功能包下新建 config 目录，config目录下新建 t2.yaml 文件，并输入如下内容：

```
/t2:
  ros__parameters:
    background_b: 0
    background_g: 0
    background_r: 50
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
```

注意，还需要在 CMakeLists.txt 中安装 config：

```
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})
```

**3.Node使用语法3**

```
turtle3 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                name="t3", 
                remappings=[("/turtle1/cmd_vel","/cmd_vel")] #话题重映射
                )
```

上述代码会创建一个 turtlesim_node 节点，并将话题名称从 /turtle1/cmd_vel 重映射到 /cmd_vel。

- remappings：话题重映射。

**4.Node使用语法4**

```
rviz = Node(package="rviz2",
                executable="rviz2",
                # 节点启动时传参
                arguments=["-d", os.path.join(get_package_share_directory("cpp01_launch"),"config","my.rviz")]
    )
```

上述代码会创建一个 rviz2 节点，并加载了 rviz2 相关的配置文件。

该配置文件可以先启动 rviz2 ，配置完毕后，保存到 config 目录并命名为 my.rviz。

- arguments：调用指令时的参数列表。

**5.Node使用语法5**

```
turtle4 = Node(package="turtlesim", 
                executable="turtlesim_node",
                # 节点启动时传参，相当于 arguments 传参时添加前缀 --ros-args 
                ros_arguments=["--remap", "__ns:=/t4_ns", "--remap", "__node:=t4"]
    )
```

上述代码会创建一个 turtlesim_node 节点，并在指令调用时传入参数列表。

- ros_arguments：相当于 arguments 前缀 --ros-args。