#ROS2

## **参数声明**

使用 `declare_parameter()` 声明节点将使用的参数

```cpp
// 声明参数并设置默认值
this->declare_parameter<std::string>("my_string", "hello");
this->declare_parameter<int>("my_int", 42);
this->declare_parameter<double>("my_double", 3.14);
this->declare_parameter<bool>("my_bool", true);
```

支持批量声明参数
```cpp
// 批量声明参数
this->declare_parameters("namespace", {
  {"param1", 10},
  {"param2", "world"},
  {"param3", 2.5}
});
```

## 参数获取

使用 `get_parameter()` 获取参数值

```cpp
// 获取参数值
std::string my_string = this->get_parameter("my_string").as_string();
int my_int = this->get_parameter("my_int").as_int();
double my_double = this->get_parameter("my_double").as_double();
bool my_bool = this->get_parameter("my_bool").as_bool();
```

批量获取参数
```cpp
// 获取所有参数
auto params = this->get_parameters({"my_string", "my_int", "my_double"});
for (const auto &param : params) {
  RCLCPP_INFO(this->get_logger(), "Parameter: %s", param.get_name().c_str());
}
```

## 参数设置

使用 `set_parameter()` 设置参数值

```cpp
// 设置新参数值
this->set_parameter(rclcpp::Parameter("my_int", 1234));
```

4. **参数验证**：通过回调函数验证参数变化的有效性

```cpp
// 创建参数变化回调
param_callback_handle_ = this->add_on_set_parameters_callback(
  std::bind(&ParamDemoNode::parametersCallback, this, std::placeholders::_1));
```

## 参数文件

支持从YAML文件加载初始参数值

创建参数文件 `params.yaml`：
```yaml
param_demo_node:
  ros__parameters:
    my_string: "from_yaml"
    my_int: 100
    my_double: 1.618
    my_bool: false
```

在启动文件中使用：
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='param_demo_node',
            name='param_demo_node',
            parameters=['path/to/params.yaml']
        ),
    ])
```



6. **命令行参数操作**
```
# 列出节点参数
ros2 param list

# 获取参数值
ros2 param get /param_demo_node my_int

# 设置参数值
ros2 param set /param_demo_node my_string "new_value"

# 导出参数到文件
ros2 param dump /param_demo_node

# 从文件加载参数
ros2 param load /param_demo_node params.yaml
```

7. CMakeLists.txt 配置

正常功能包配置，参考[[ROS2功能包#CMakeLists]]

## 命令行启动节点时配置参数

使用 `--ros-args` 命令行参数传递 ROS 参数
在 ROS 中，可以通过命令行选项 `--ros-args` 来设置和配置节点的参数。
这使得开发者能够在启动节点时动态调整其行为而无需修改源代码

基本形式如下：
```
ros2 run package_name executable_name --ros-args [options...]
```

e.g. ：

1. 指定单个参数：
```
ros2 run demo_nodes_cpp talker --ros-args --param period_ms:=500
```


2. 加载外部文件中的参数：
当有大量复杂的配置项时，推荐使用 `YAML` 文件保存它们，并通过 `-p/--params-file` 加载进来
```
ros2 run demo_nodes_cpp listener --ros-args --params-file path/to/config.yaml

```


3. 组合使用多条指令：
```
ros2 run image_tools showimage \
  --ros-args \
    --remap __node:=my_image_viewer \
    --param auto_resize:=true \
    --log-level WARN
```