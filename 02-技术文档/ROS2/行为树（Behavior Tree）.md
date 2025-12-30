#行为树
### 安装BehaviorTree.CPP
```
# 安装 BehaviorTree.CPP（ROS2常用的行为树库）
sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp
```

从GitHub安装
```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
```
  
使用 CMake 构建项目
```
mkdir build
cd build
cmake ..
make
```

## 超详细行为树讲解
https://blog.csdn.net/yyh0201/article/details/142522659?fromshare=blogdetail&sharetype=blogdetail&sharerId=142522659&sharerefer=PC&sharesource=qihemu_&sharefrom=from_link



# BehaviorTree.CPP的使用

由deepseek生成的例程，[[行为树例程 form DeepSeek]]

## 节点定义

使用类继承`BehaviorTree.CPP`内部类
### 同步动作节点
继承自类`BT::SyncActionNode`

```cpp
/**
 * @brief 获得随机数
 * 
 */
class GetRanNum: public BT::SyncActionNode
{
  public:
    GetRanNum(const std::string& name, const BT::NodeConfig& config):
        SyncActionNode(name, config)
    {}
 
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("max"),
            BT::InputPort<double>("min"),
            BT::OutputPort<double>("random_num")
        };
    }

    BT::NodeStatus tick() override
    {
        double max, min;
        if (!getInput<double>("max", max)) return BT::NodeStatus::FAILURE;
        if (!getInput<double>("min", min)) return BT::NodeStatus::FAILURE;

        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<double> distr(min, max);

        double random_num = distr(eng);
        printf("[GetRanNum]. get random num:%f\n", random_num);

        setOutput<double>("random_num", random_num);
        return BT::NodeStatus::SUCCESS;
    }
};

```



### 持续动作节点
继承自类`BT::StatefulActionNode`

```cpp
/**
* @brief 移动动作节点
*
*/
class MoveInPlace : public BT::StatefulActionNode
{
public:
	// 任何带有端口的TreeNode都必须有一个带有此签名的构造函数
	MoveInPlace(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<TurtleROSNode> turtle_node)
	: StatefulActionNode(name, config), turtle_node_(turtle_node)
	{}
	
	// 必须定义这个静态方法
	static BT::PortsList providedPorts()
	{
		return{ BT::InputPort<double>("target_dist") };
	}
	
	// 这个函数在开始时调用一次
	BT::NodeStatus onStart() override;
	
	// 如果onStart()返回RUNNING，我们将继续调用此方法，直到它返回与RUNNING不同的结果
	BT::NodeStatus onRunning() override;
	
	// 当操作被另一个节点中止时执行的回调
	void onHalted() override;
	  
private:
	double target_dist_;
	std::shared_ptr<TurtleROSNode> turtle_node_; // 从ROS类里调用参数
	bool first_time_ = true;
};
```

以上`onStart()`、`onRunning()`、`onHalted()`三个函数必须复写
例子如下：

```cpp
// 在开始时调用一次
BT::NodeStatus MoveInPlace::onStart()
{
	// 获得输入端口数据
    if ( !getInput<double>("target_dist", target_dist_))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }
    
    // 调用成功则返回RUNNING开始节点动作
    return BT::NodeStatus::RUNNING;
}
```


```cpp
// 如果onStart()返回RUNNING，我们将继续调用此方法，直到它返回与RUNNING不同的结果
// 该函数将持续运行，知道返回值不是RUNNING
BT::NodeStatus MoveInPlace::onRunning()
{
	// 省略动作逻辑...
	
    if (current_dist < 0.2)
    {
        first_time_ = true;
        printf("[MoveInPlace]. finished.\n");
        
        // 动作结束返回SUCCESS
        return BT::NodeStatus::SUCCESS;
    }
    
	// 省略动作逻辑...
    
    // 过程中持续返回RUNNING
    return BT::NodeStatus::RUNNING;
}
```


```cpp
// 当操作被另一个节点中止时执行的回调
void MoveInPlace::onHalted()
{
    printf("[MoveInPlace]. onHalted.\n");
}
```


### 条件节点
继承自类`BT::ConditionNode`


```cpp
/**
 * @brief 条件节点，边界检测
 * 
 */
class CheckBoundary : public BT::ConditionNode
{
public:
    CheckBoundary(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<TurtleROSNode> turtle_node)
        : ConditionNode(name, config), turtle_node_(turtle_node)
    {}

	// 定义静态端口列表
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("safe_margin", 0.5, "安全边界距离"),
            BT::InputPort<double>("min_x", 0.0, "最小X坐标"),
            BT::InputPort<double>("max_x", 11.0, "最大X坐标"),
            BT::InputPort<double>("min_y", 0.0, "最小Y坐标"),
            BT::InputPort<double>("max_y", 11.0, "最大Y坐标")
        };
    }
    
    // 复写tick，主要的判断逻辑在这里
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<TurtleROSNode> turtle_node_;
};
```

复写`tick()`
主要的条件判断逻辑在这里
```cpp

BT::NodeStatus CheckBoundary::tick()
{
    double safe_margin, min_x, max_x, min_y, max_y;
  
    // 获取输入参数，如果未设置则使用默认值
    if (!getInput("safe_margin", safe_margin)) safe_margin = 0.5;
    if (!getInput("min_x", min_x)) min_x = 0.0;
    if (!getInput("max_x", max_x)) max_x = 11.0;
    if (!getInput("min_y", min_y)) min_y = 0.0;
    if (!getInput("max_y", max_y)) max_y = 11.0;

	// 省略判断逻辑...
	
	// 根据条件返回SUCCESS或是FAILURE
    if (near_boundary)
    {
        return BT::NodeStatus::FAILURE; 
    }
    return BT::NodeStatus::SUCCESS; 
}
```



## 注册节点
定义了的节点需要注册后才能在`XML`文件中调用

```cpp
// 创建行为树工厂
BT::BehaviorTreeFactory factory;

// 注册自定义节点
factory.registerNodeType<GetRanNum>("GetRanNum");
factory.registerNodeType<MoveInPlace>("MoveInPlace", turtle_node);
factory.registerNodeType<RotateInPlace>("RotateInPlace", turtle_node);
factory.registerNodeType<CheckPositionCondition>("CheckPositionCondition", turtle_node);
```
读取`XML`文件中的树结构
```cpp
// 创建行为树（使用XML定义）
auto tree = factory.createTreeFromFile("./turtle_tree.xml");
```

## 行为树关键字

序列
```xml
<Sequence>
```

RetryUntilSuccessful，`num_attempts`设置尝试次数
```xml
<RetryUntilSuccessful num_attempts="5">
	<Delay delay_msec="100">
		<CheckArmStates/>
	</Delay>
</RetryUntilSuccessful>
```

延时，`delay_msec`设置延时时间，ms
```xml
<Delay delay_msec="100">
	<CheckArmStates/>
</Delay>
```



## 黑板的使用

创建黑板对象
```cpp
auto blackboard = config().blackboard;
```

从黑板获取
```cpp
blackboard->get<bool>("IsAppleExisted", is_apple_existed)
```
通常使用`if`判断是否获取成功
```cpp
// 尝试从黑板读取变量
if (!blackboard->get<bool>("IsAppleExisted", is_apple_existed))
{
	// 如果读取失败则表示目前没有Apple
	is_apple_existed = false;
}
```

向黑板设置
```cpp
blackboard->set<bool>("IsAppleExisted", true);
blackboard->set<double>("AppleX", apple_x);
blackboard->set<double>("AppleY", apple_y);
```




## 执行行为树

- 不集成ROS的情况
直接tick，该方法阻塞程序
```cpp
tree.tickWhileRunning();
```

- 集成ROS或是其他
```cpp
// 执行行为树
while (rclcpp::ok())
{
	// spin_some，处理ros消息
	rclcpp::spin_some(turtle_node);
	
	// tickOnce，执行行为树
	BT::NodeStatus status = tree.tickOnce();
	
	if (status == BT::NodeStatus::RUNNING)
	{
		// 延时
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	else
	{
		RCLCPP_INFO(turtle_node->get_logger(), "行为树执行完成");
		break;
	}
}
```




# Groot2
## 安装groot2

下载地址：
https://www.behaviortree.dev/groot/

放到`~`的位置，运行以下命令
```
cd ~
chmod +x Groot2-1.0.1-linux-installer.run
./Groot2-1.0.1-linux-installer.run
```

安装完成后，输入：
```
cd ~/Groot2/bin
./groot2
```

## 使用groot2

- 从C++程序导出自定义节点模型
从`factory`创建node模型，并生成xml文件

```cpp

// 注册自定义节点...

// 创建行为树（使用XML定义）
auto tree = factory.createTreeFromFile("./turtle_tree.xml");

// 生成XML
// 包含于头文件 "behaviortree_cpp/xml_parsing.h"
std::string xml_content = BT::writeTreeNodesModelXML(factory);

// 写入文件
std::ofstream file("turtle_bt_nodes.xml");
file << xml_content;
file.close();
```

生成的`turtle_bt_nodes.xml`文件就是groot2读取自定义节点所需的xml文件



