# Robot Control 模块完整说明文档

> **作者说明**：本文档面向初学者，将用通俗易懂的语言和大量示例帮助您理解按摩机器人的控制系统。  
> **文档版本**：v1.0  
> **创建日期**：2026年1月22日

---

## 📖 目录

1. [整体架构概览](#1-整体架构概览)
2. [核心概念解释](#2-核心概念解释)
3. [状态机详细解析](#3-状态机详细解析)
4. [行为树系统](#4-行为树系统)
5. [按摩技法节点详解](#5-按摩技法节点详解)
6. [数据流转过程](#6-数据流转过程)
7. [实际执行示例](#7-实际执行示例)
8. [常见问题解答](#8-常见问题解答)

---

## 1. 整体架构概览

### 1.1 这个模块是做什么的？

想象一下，您去按摩店，按摩师会：
1. **先准备好**：检查设备、洗手、准备精油
2. **问您需求**：哪里不舒服？力度多大？
3. **开始按摩**：按照不同手法给您按摩
4. **完成服务**：收拾工具、送您离开

我们的`robot_control`模块就像是按摩机器人的"大脑"，负责控制机器人完成上述所有流程。

### 1.2 系统架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    MassageRobotStateMachine                 │
│                      (按摩机器人状态机)                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 包含
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                         五大主状态                            │
├──────────┬──────────┬──────────┬──────────┬────────────────┤
│ MsInit   │ MsIdle   │MsPrepar  │MsExecu   │   MsError      │
│(初始化)   │(待机)     │(准备)     │(执行)     │  (错误处理)     │
└──────────┴──────────┴──────────┴──────────┴────────────────┘
                              │
                              │ 使用
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    十大正交线(Orthogonal)                     │
├────────────┬────────────┬────────────┬────────────┬────────┤
│OrUserSvc   │OrMonitor   │OrTimer     │OrArmCtrl   │ ...    │
│(用户服务)   │(监控)       │(定时器)     │(机械臂)     │ (其他)  │
└────────────┴────────────┴────────────┴────────────┴────────┘
                              │
                              │ 执行
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                     行为树 (Behavior Tree)                   │
├───────────────┬───────────────┬───────────────┬────────────┤
│穴位点按        │旋转推揉         │螺旋移动         │  ...       │
│(Press)        │(Twist)         │(Spiral)        │  (其他技法) │
└───────────────┴───────────────┴───────────────┴────────────┘
```

### 1.3 核心组成部分

```cpp
// 这是状态机的主体定义 (massage_robot_sm.hpp)
struct MassageRobotStateMachine 
    : public smacc2::SmaccStateMachineBase<MassageRobotStateMachine, MsInit>
{
    virtual void onInitialize() override
    {
        // 创建10个正交线，它们可以并行工作
        this->createOrthogonal<OrUserService>();      // 1. 用户服务
        this->createOrthogonal<OrMonitor>();          // 2. 监控
        this->createOrthogonal<OrTimer>();            // 3. 定时器
        this->createOrthogonal<OrArmControl>();       // 4. 机械臂控制
        this->createOrthogonal<OrCameraControl>();    // 5. 相机控制
        this->createOrthogonal<OrRecognition>();      // 6. 识别
        this->createOrthogonal<OrToolControl>();      // 7. 工具控制
        this->createOrthogonal<OrSystemInitCheck>();  // 8. 系统自检
        this->createOrthogonal<OrDataLoader>();       // 9. 数据加载
        this->createOrthogonal<OrRiskManagement>();   // 10. 风控管理
    }
};
```

---

## 2. 核心概念解释

### 2.1 什么是状态机(State Machine)？

**生活中的例子**：想象一个红绿灯

```
     红灯 ──30秒──> 绿灯 ──20秒──> 黄灯
      ▲                              │
      └──────────3秒─────────────────┘
```

红绿灯有三种**状态**（红、绿、黄），在特定**事件**（时间到）发生时会**转换**到另一个状态。

**机器人的状态机**：

```
系统启动 ──检测完成──> 待机中 ──开始按摩──> 按摩中 ──完成──> 待机中
   │                     │                    │
   └────错误────> 错误处理 <────错误───────────┘
```

### 2.2 什么是正交线(Orthogonal)？

**生活中的例子**：您在开车时

```
正交线1: 控制方向盘  (左转、右转、直行)
正交线2: 控制油门    (加速、减速、停止)
正交线3: 播放音乐    (播放、暂停、切歌)
```

这三件事可以**同时进行**，互不影响！这就是正交线的概念。

**机器人的正交线**：

```
OrArmControl     : 控制机械臂运动
OrToolControl    : 控制按摩头工具
OrMonitor        : 监控安全信号
OrTimer          : 管理定时器
... 等等，它们都可以并行工作！
```

### 2.3 什么是行为树(Behavior Tree)？

**生活中的例子**：早上起床的流程

```
起床流程
├── 顺序执行：
│   ├── 关闹钟 ✓
│   ├── 刷牙   ✓
│   ├── 洗脸   ✓
│   └── 吃早餐 ✓
```

如果任何一步失败（比如没水洗脸），整个流程就会停止。

**机器人的行为树**：

```
按摩一个穴位
├── 顺序执行：
│   ├── 移动到穴位上方 ✓
│   ├── 接触穴位      ✓
│   ├── 施加压力      ✓
│   ├── 保持3秒       ✓
│   └── 抬起          ✓
```

---

## 3. 状态机详细解析

### 3.1 五大主状态概览

按摩机器人的一生分为5个主要阶段：

```
┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│ MsInit   │──>│ MsIdle   │──>│MsPrepar  │──>│MsExecu   │──>│ MsIdle   │
│(初始化)   │   │(待机)     │   │(准备)     │   │(执行)     │   │(待机)     │
└──────────┘   └──────────┘   └──────────┘   └──────────┘   └──────────┘
      │              │              │              │
      └──────────────┴──────────────┴──────────────┴────────> MsError
                                                            (任何错误都会跳转到这里)
```

### 3.2 状态1：MsInit (初始化状态)

#### 作用
系统开机后的第一个状态，确保所有硬件和软件都正常工作。

#### 子状态流程

```
MsInit (初始化阶段)
  │
  ├─> StHardwareCheck    (硬件检测)
  │    ├─ 检查机械臂连接
  │    ├─ 检查相机连接
  │    ├─ 检查传感器状态
  │    └─ 成功 ──> 下一步
  │
  ├─> StSoftwareCheck    (软件检测)
  │    ├─ 检查ROS节点
  │    ├─ 检查话题连接
  │    └─ 成功 ──> 下一步
  │
  ├─> StDataInit         (数据初始化)
  │    ├─ 加载配置文件
  │    ├─ 加载按摩方案
  │    └─ 成功 ──> 下一步
  │
  ├─> StReturnToHome     (回到原点)
  │    ├─ 机械臂归位
  │    └─ 成功 ──> 下一步
  │
  └─> StMotionTest       (运动测试)
       ├─ 执行简单动作测试
       └─ 成功 ──> 发送 EvInitComplete 事件
                    │
                    └──> 转换到 MsIdle (待机)
```

#### 代码示例

```cpp
// MsInit 状态定义 (ms_init.hpp)
struct MsInit : smacc2::SmaccState<MsInit, MassageRobotStateMachine, StHardwareCheck>
{
    // 定义状态转换表
    typedef mpl::list<
        // 全局急停
        Transition<EvEmergencyStop, MsError>,
        
        // 正常流程
        Transition<EvInitComplete, MsIdle>,          // 初始化完成 → 待机
        Transition<EvHardwareCheckFailed, MsError>,  // 硬件失败 → 错误
        Transition<EvSoftwareCheckFailed, MsError>,  // 软件失败 → 错误
        Transition<EvReturnHomeFailed, MsError>      // 归位失败 → 错误
    > reactions;

    void onEntry() 
    { 
        RCLCPP_INFO(getLogger(), "进入系统初始化阶段 (MsInit)");
    }
};
```

#### 实际执行例子

假设系统开机：

1. **进入硬件检测子状态**
   ```
   [INFO] 进入系统初始化阶段 (MsInit)
   [INFO] 开始硬件检测...
   [INFO] ✓ 机械臂连接正常
   [INFO] ✓ 相机连接正常
   [INFO] ✓ 力传感器正常
   ```

2. **如果硬件检测失败**
   ```
   [ERROR] ✗ 机械臂连接失败
   [INFO] 发送事件: EvHardwareCheckFailed
   [INFO] 状态转换: MsInit → MsError
   ```

3. **如果全部成功**
   ```
   [INFO] ✓ 所有检测通过
   [INFO] 发送事件: EvInitComplete
   [INFO] 状态转换: MsInit → MsIdle
   ```

---

### 3.3 状态2：MsIdle (待机状态)

#### 作用
系统处于待命状态，等待用户开始按摩，同时可以播放音乐、显示呼吸灯等。

#### 子状态流程

```
MsIdle (待机阶段)
  │
  ├─> StIdleArmHoming    (机械臂归位)
  │    ├─ 移动到安全位置
  │    └─ 完成 ──> 下一步
  │
  ├─> StIdle             (等待用户指令)
  │    ├─ 播放呼吸灯
  │    ├─ 播放欢迎音乐
  │    └─ 等待用户选择...
  │         │
  │         ├─ 用户点击"开始拍照准备" 
  │         │   └──> 发送 EvMassageLifeCyclePhotoTakeReady
  │         │         └──> 转换到 MsPreparation
  │         │
  │         └─ 用户点击"打包模式"
  │             └──> 进入 StPackaging 子状态
  │
  └─> StPackaging        (打包模式 - 机械臂收纳)
       └─ 完成 ──> StPackagingCompleted
```

#### 代码示例

```cpp
// MsIdle 状态定义 (ms_idle.hpp)
struct MsIdle : smacc2::SmaccState<MsIdle, MassageRobotStateMachine, StIdleArmHoming>
{
    typedef mpl::list<
        // 全局急停
        Transition<EvEmergencyStop, MsError>,
        
        // 机械臂保护性停止
        Transition<EvArmProtectiveStop, MsError>,
        
        // 正常流程：用户请求拍照准备
        Transition<EvMassageLifeCyclePhotoTakeReady, MsPreparation>
    > reactions;

    void onEntry() 
    { 
        RCLCPP_INFO(getLogger(), "进入系统待机阶段 (MsIdle)");
        // 这里可以触发播放欢迎音乐、显示呼吸灯等
    }
};
```

#### 实际执行例子

```
时间轴：
00:00 [INFO] 进入系统待机阶段 (MsIdle)
00:01 [INFO] 机械臂归位中...
00:05 [INFO] 机械臂已归位，进入待机状态
00:06 [INFO] 播放呼吸灯效果...
00:07 [INFO] 播放欢迎音乐："您好，欢迎使用按摩机器人"

... 用户在屏幕上选择按摩部位（背部）...

00:45 [INFO] 用户请求：开始拍照准备
00:45 [INFO] 发送事件: EvMassageLifeCyclePhotoTakeReady
00:45 [INFO] 状态转换: MsIdle → MsPreparation
```

---

### 3.4 状态3：MsPreparation (准备状态)

#### 作用
进行按摩前的准备工作：拍照、识别穴位、加载按摩方案。

#### 子状态流程

```
MsPreparation (准备阶段)
  │
  ├─> StPhotoTakeReady              (拍照就绪)
  │    ├─ 机械臂移动到拍照位置
  │    ├─ 相机准备
  │    └─ 等待用户确认拍照...
  │         │
  │         └─ 用户点击"拍照" 
  │             └──> 发送 EvMassageTakePhotoRecognizeReq
  │                   └──> 转到 StPhotoTakenRecognizing
  │
  ├─> StPhotoTakenRecognizing       (拍照识别中)
  │    ├─ 拍照
  │    ├─ AI识别穴位位置
  │    ├─ 坐标转换（图片坐标 → 机器人坐标）
  │    └─ 识别结果：
  │         ├─ 成功 ──> StPhotoTakenRecognizeSucceed
  │         └─ 失败 ──> StPhotoTakenRecognizeFailed
  │
  ├─> StPhotoTakenRecognizeSucceed  (识别成功)
  │    ├─ 显示识别到的穴位
  │    ├─ 加载对应按摩方案
  │    └─ 等待用户确认开始...
  │         │
  │         └─ 用户点击"开始按摩"
  │             └──> 发送 EvMassageStartReq
  │                   └──> 转到 MsExecution
  │
  └─> StPhotoTakenRecognizeFailed   (识别失败)
       ├─ 显示错误信息
       └─ 提示用户重新拍照或返回
```

#### 代码示例

```cpp
// MsPreparation 状态定义 (ms_preparation.hpp)
struct MsPreparation : smacc2::SmaccState<MsPreparation, MassageRobotStateMachine, StPhotoTakeReady>
{
    typedef mpl::list<
        // 全局急停
        Transition<EvEmergencyStop, MsError>,
        
        // 机械臂急停
        Transition<EvArmEmergencyStop, MsError>,
        
        // 用户取消准备
        Transition<EvMassagePrepareCancelReq, MsIdle>
    > reactions;

    void onEntry() 
    { 
        RCLCPP_INFO(getLogger(), "进入按摩准备阶段 (MsPreparation)");
    }
};
```

#### 实际执行例子

**场景：用户选择按摩背部**

```
00:45 [INFO] 进入按摩准备阶段 (MsPreparation)
00:45 [INFO] 子状态: StPhotoTakeReady (拍照就绪)
00:46 [INFO] 机械臂移动到拍照位置...
00:50 [INFO] 相机已准备，请用户调整姿势
00:51 [TTS] "请保持静止，即将拍照"

... 用户点击"拍照"按钮...

00:55 [INFO] 用户请求：开始拍照识别
00:55 [INFO] 子状态转换: StPhotoTakeReady → StPhotoTakenRecognizing
00:56 [INFO] 正在拍照...
00:57 [INFO] 正在AI识别穴位...
00:58 [INFO] 识别成功！检测到5个穴位：
              - 大椎穴 (C7)
              - 风门穴 (BL12)
              - 肺俞穴 (BL13)
              - 心俞穴 (BL15)
              - 膈俞穴 (BL17)
00:59 [INFO] 子状态转换: StPhotoTakenRecognizing → StPhotoTakenRecognizeSucceed
01:00 [INFO] 已加载背部按摩方案
01:00 [TTS] "识别完成，请确认开始按摩"

... 用户点击"开始按摩"...

01:15 [INFO] 用户请求：开始按摩
01:15 [INFO] 发送事件: EvMassageStartReq
01:15 [INFO] 状态转换: MsPreparation → MsExecution
```

---

### 3.5 状态4：MsExecution (执行状态)

#### 作用
**这是最核心的状态！** 机器人在这里真正执行按摩动作。

#### 子状态流程

```
MsExecution (执行阶段)
  │
  ├─> StMassageWarmUp           (预热阶段)
  │    ├─ 轻柔的热身动作
  │    ├─ 让用户适应
  │    └─ 完成 ──> StMassaging
  │
  ├─> StMassaging               (按摩中)
  │    │
  │    ├─ 执行行为树 (Behavior Tree)
  │    │    │
  │    │    ├─ 段落1: 重复3次
  │    │    │    ├─ 技法1: 穴位点按 (大椎穴)
  │    │    │    ├─ 技法2: 旋转推揉 (风门穴)
  │    │    │    └─ 技法3: 螺旋移动 (肺俞穴)
  │    │    │
  │    │    ├─ 段落2: 重复2次
  │    │    │    ├─ 技法4: 环形按压
  │    │    │    └─ 技法5: 正弦波动
  │    │    │
  │    │    └─ ... 继续执行...
  │    │
  │    ├─ 用户操作：
  │    │    ├─ 点击"暂停" ──> StPaused (暂停状态)
  │    │    ├─ 点击"取消" ──> StMassageSucceed (提前结束)
  │    │    └─ 工具需要更换 ──> SsToolChange (工具更换子状态机)
  │    │
  │    └─ 完成所有技法 ──> StMassageSucceed
  │
  ├─> StPaused                  (暂停状态)
  │    ├─ 机械臂停止运动
  │    ├─ 保持当前位置
  │    └─ 等待用户操作：
  │         ├─ 点击"继续" ──> StMassaging
  │         └─ 点击"取消" ──> StMassageSucceed
  │
  ├─> SsToolChange              (工具更换子状态机)
  │    ├─ 提示更换工具
  │    ├─ 等待工具更换完成
  │    └─ 完成 ──> StMassaging (继续按摩)
  │
  ├─> StMassageSucceed          (按摩成功完成)
  │    ├─ 机械臂归位
  │    ├─ 保存按摩记录
  │    └─ 转换到 MsIdle
  │
  └─> StMassageFailed           (按摩失败)
       ├─ 记录错误信息
       └─ 转换到 MsError
```

#### 代码示例

```cpp
// MsExecution 状态定义 (ms_execution.hpp)
struct MsExecution : smacc2::SmaccState<MsExecution, MassageRobotStateMachine, StMassageWarmUp>
{
    typedef mpl::list<
        // 全局急停
        Transition<EvEmergencyStop, MsError>,
        
        // 机械臂急停
        Transition<EvArmEmergencyStop, MsError>,
        
        // 风险紧急停止（如压力过大）
        Transition<EvRiskEmergencyStop, MsError>,
        
        // 用户取消按摩（转到成功状态进行清理）
        Transition<EvMassageCancelReq, StMassageSucceed>
    > reactions;

    void onEntry() 
    { 
        RCLCPP_INFO(getLogger(), "进入按摩执行阶段 (MsExecution)");
    }
};
```

#### 实际执行例子

**完整的按摩执行过程：**

```
01:15 [INFO] 进入按摩执行阶段 (MsExecution)
01:15 [INFO] 子状态: StMassageWarmUp (预热阶段)
01:16 [TTS] "即将开始按摩，请放松身体"
01:17 [INFO] 执行预热动作...
01:20 [INFO] 预热完成
01:20 [INFO] 子状态转换: StMassageWarmUp → StMassaging

01:20 [INFO] 开始执行按摩行为树
01:20 [INFO] ========== 段落1 (共3次重复) ==========
01:21 [INFO] --- 第1次重复 ---
01:21 [INFO] 技法1: 穴位点按 (大椎穴)
01:21 [DEBUG] 移动到穴位上方... 
01:22 [DEBUG] 接触穴位...
01:23 [DEBUG] 施加压力: 150N, 持续3秒
01:26 [DEBUG] 抬起...
01:27 [INFO] ✓ 技法1完成

01:27 [INFO] 技法2: 旋转推揉 (风门穴)
01:28 [DEBUG] 移动到风门穴...
01:29 [DEBUG] 接触并开始旋转，角度: ±30°, 频率: 0.5Hz
01:34 [DEBUG] 旋转完成，抬起
01:35 [INFO] ✓ 技法2完成

01:35 [INFO] 技法3: 螺旋移动 (肺俞穴)
01:36 [DEBUG] 从肺俞穴向下螺旋移动10cm...
01:41 [DEBUG] 螺旋移动完成
01:42 [INFO] ✓ 技法3完成

01:42 [INFO] --- 第2次重复 ---
01:42 [INFO] 技法1: 穴位点按 (大椎穴)
... (重复相同过程) ...

... 用户点击"暂停"按钮...

05:30 [INFO] 用户请求: 暂停按摩
05:30 [INFO] 发送事件: EvMassagePauseReq
05:30 [INFO] 子状态转换: StMassaging → StPaused
05:31 [INFO] 机械臂已停止，当前位置保持
05:31 [TTS] "按摩已暂停，点击继续可恢复"

... 1分钟后，用户点击"继续"...

06:35 [INFO] 用户请求: 恢复按摩
06:35 [INFO] 发送事件: EvMassageResumeReq
06:35 [INFO] 子状态转换: StPaused → StMassaging
06:36 [INFO] 继续执行按摩...

... 所有技法执行完毕...

15:20 [INFO] 所有按摩技法已完成！
15:20 [INFO] 子状态转换: StMassaging → StMassageSucceed
15:21 [INFO] 机械臂归位中...
15:25 [INFO] 保存按摩记录到数据库...
15:26 [TTS] "按摩已完成，感谢使用！"
15:26 [INFO] 状态转换: MsExecution → MsIdle
```

---

### 3.6 状态5：MsError (错误处理状态)

#### 作用
处理任何异常情况，确保系统安全。

#### 错误类型

```
MsError 可以处理的错误：
  │
  ├─ 急停错误
  │   ├─ 用户按下急停按钮
  │   └─ 处理: 立即停止所有运动
  │
  ├─ 硬件错误
  │   ├─ 机械臂通信中断
  │   ├─ 相机连接断开
  │   └─ 处理: 尝试重连或等待修复
  │
  ├─ 安全错误
  │   ├─ 压力传感器检测到异常压力
  │   ├─ 碰撞检测触发
  │   └─ 处理: 立即停止，抬起机械臂
  │
  └─ 其他错误
      ├─ 轨迹规划失败
      ├─ 行为树执行错误
      └─ 处理: 记录日志，等待人工介入
```

#### 实际执行例子

**场景1：用户按下急停按钮**

```
05:30 [INFO] 正在执行技法3...
05:31 [WARN] 检测到急停信号！
05:31 [ERROR] 发送事件: EvEmergencyStop
05:31 [ERROR] 状态转换: MsExecution → MsError
05:31 [INFO] 进入错误处理状态
05:31 [ACTION] 立即停止所有运动
05:32 [ACTION] 机械臂安全停止
05:32 [ACTION] 记录错误日志
05:32 [TTS] "检测到急停，系统已安全停止"
05:33 [INFO] 等待用户解除急停并重启系统...
```

**场景2：压力传感器检测到异常**

```
08:15 [INFO] 正在施加压力...
08:16 [WARN] 力传感器读数异常: 当前压力 350N (阈值: 300N)
08:16 [ERROR] 发送事件: EvRiskEmergencyStop
08:16 [ERROR] 状态转换: MsExecution → MsError
08:16 [INFO] 风控紧急停止触发
08:17 [ACTION] 立即抬起机械臂
08:18 [TTS] "检测到压力异常，按摩已停止"
08:18 [INFO] 等待检查和重启...
```

---

## 4. 行为树系统

### 4.1 什么是行为树？

行为树是一种**任务执行框架**，把复杂的任务分解成一个个小节点，然后按照树状结构执行。

**生活中的例子：做一道菜**

```
做宫保鸡丁 (顺序执行)
├─ 准备食材
│   ├─ 切鸡肉 ✓
│   ├─ 准备花生 ✓
│   └─ 准备调料 ✓
├─ 烹饪
│   ├─ 热锅 ✓
│   ├─ 炒鸡肉 ✓
│   ├─ 加调料 ✓
│   └─ 翻炒 ✓
└─ 装盘 ✓
```

如果任何一步失败（比如没有花生），整个菜就做不成。

### 4.2 按摩行为树结构

```
按摩方案 (Root)
│
├─ 段落1 (重复3次)
│   ├─ TimedMassageLoopNode (定时循环节点)
│   │   └─ 执行时间: 5分钟
│   │       └─ 包含的技法：
│   │           ├─ TechAcupointPressNode (穴位点按)
│   │           ├─ TechAcupointTwistNode (穴位旋转)
│   │           └─ TechAcupointSpiralMoveNode (螺旋移动)
│   │
│   └─ 每个技法节点包含：
│       ├─ 穴位信息
│       ├─ 力度参数
│       ├─ 速度参数
│       └─ 重复次数
│
├─ 段落2 (重复2次)
│   └─ ...
│
└─ 段落3 (重复1次)
    └─ ...
```

### 4.3 行为树节点类型

robot_control模块中有多种按摩技法节点：

| 节点名称 | 功能 | 参数 |
|---------|------|------|
| `TechAcupointPressNode` | 穴位点按 | 穴位、压力、持续时间 |
| `TechAcupointTwistNode` | 穴位旋转 | 穴位、角度、频率 |
| `TechAcupointSpiralMoveNode` | 螺旋移动 | 起点、终点、螺距 |
| `TechCircularPressNode` | 环形按压 | 中心点、半径、圈数 |
| `TechRotaryFingerTherapyNode` | 旋转指压 | 穴位、旋转角度 |
| `TechSineWaveNode` | 正弦波动 | 振幅、频率、距离 |
| `TechGentleMoveToNode` | 柔和移动 | 起点、终点、加减速 |
| `TechContactMoveToNode` | 接触移动 | 起点、终点、接触力 |
| `TimedMassageLoopNode` | 定时循环 | 持续时间、子节点 |

---

## 5. 按摩技法节点详解

### 5.1 技法节点的生命周期

每个技法节点都有三个主要阶段：

```
┌─────────────┐
│  onStart()  │  <-- 节点开始执行
└──────┬──────┘
       │
       ├─> 初始化参数
       ├─> 获取穴位坐标
       ├─> 创建Action目标
       └─> 发送Action请求
       │
┌──────▼──────────┐
│  onRunning()    │  <-- 节点执行中（不断被调用）
└──────┬──────────┘
       │
       ├─> 检查Action状态
       ├─> 监听反馈信息
       ├─> 检查是否需要取消
       └─> 返回 RUNNING / SUCCESS / FAILURE
       │
┌──────▼──────┐
│  onHalted() │  <-- 节点被中止
└─────────────┘
       │
       └─> 清理资源
```

### 5.2 案例1：穴位点按节点 (TechAcupointPressNode)

这是最基础的按摩手法，就像用手指按压穴位。

#### 执行流程

```
穴位点按 (Press)
│
├─ 步骤1: 移动到穴位上方 (hover_height = 5cm)
│   └─ 轨迹: 当前位置 → 穴位上方
│
├─ 步骤2: 下降接触穴位
│   └─ 轨迹: 上方位置 → 穴位位置
│
├─ 步骤3: 施加压力并保持
│   ├─ 启动力控: 目标压力 = 150N
│   ├─ 沿Z轴施加压力
│   └─ 保持时间: 3秒
│
└─ 步骤4: 抬起
    └─ 轨迹: 穴位位置 → 上方位置
```

#### 关键代码解析

```cpp
// 来自: tech_acupoint_press_node.cpp

BT::NodeStatus TechAcupointPressNode::onStart()
{
    // 1. 获取穴位坐标
    Acupoint3DPointList acupoint_list;
    getInput<Acupoint3DPointList>("acupoint_points", acupoint_list);
    std::array<double, 6> start_pose = {
        acupoint_list.points[0].coordinates[0],  // x
        acupoint_list.points[0].coordinates[1],  // y
        acupoint_list.points[0].coordinates[2],  // z
        acupoint_list.points[0].coordinates[3],  // rx
        acupoint_list.points[0].coordinates[4],  // ry
        acupoint_list.points[0].coordinates[5]   // rz
    };
    
    // 2. 创建轨迹点序列
    std::vector<Waypoint> waypoints;
    
    // 2.1 移动到上方
    Waypoint hover_point;
    hover_point.pose = start_pose;
    hover_point.pose[2] += hover_height_;  // Z轴抬高5cm
    hover_point.velocity = speed_;
    hover_point.blend_radius = 0.01;
    waypoints.push_back(hover_point);
    
    // 2.2 下降到穴位
    Waypoint contact_point;
    contact_point.pose = start_pose;
    contact_point.velocity = speed_;
    contact_point.blend_radius = 0.0;
    waypoints.push_back(contact_point);
    
    // 2.3 施加压力（保持位置不变，启动力控）
    Waypoint press_point;
    press_point.pose = start_pose;
    press_point.velocity = 0.0;  // 停止不动
    press_point.blend_radius = 0.0;
    press_point.force_control_enabled = true;  // 启动力控
    press_point.target_force_z = pressure_;    // 目标压力150N
    press_point.hold_time = 3.0;               // 保持3秒
    waypoints.push_back(press_point);
    
    // 2.4 抬起
    Waypoint lift_point;
    lift_point.pose = start_pose;
    lift_point.pose[2] += hover_height_;
    lift_point.velocity = speed_;
    lift_point.blend_radius = 0.0;
    lift_point.force_control_enabled = false;  // 关闭力控
    waypoints.push_back(lift_point);
    
    // 3. 发送Action请求
    auto goal_msg = WaypointAction::Goal();
    goal_msg.waypoints = waypoints;
    action_client_->async_send_goal(goal_msg);
    
    return BT::NodeStatus::RUNNING;
}
```

#### 实际执行例子

```
[INFO] 开始执行: 穴位点按 (大椎穴)
[DEBUG] 当前机械臂位置: [0.3, 0.2, 0.5, 0, 0, 0]
[DEBUG] 目标穴位位置: [0.4, 0.25, 0.35, 0, 0, 0]

00:00 [DEBUG] 阶段1: 移动到穴位上方
00:00 [DEBUG] 目标: [0.4, 0.25, 0.40, 0, 0, 0] (Z+5cm)
00:00 [DEBUG] 速度: 0.2 m/s
00:02 [DEBUG] ✓ 到达上方位置

00:02 [DEBUG] 阶段2: 下降接触穴位
00:02 [DEBUG] 目标: [0.4, 0.25, 0.35, 0, 0, 0]
00:03 [DEBUG] ✓ 已接触穴位

00:03 [DEBUG] 阶段3: 施加压力
00:03 [DEBUG] 启动力控模式
00:03 [DEBUG] 目标压力: 150N (Z轴向下)
00:03 [DEBUG] 当前压力: 120N
00:04 [DEBUG] 当前压力: 145N
00:05 [DEBUG] 当前压力: 150N ✓ 达到目标
00:05 [DEBUG] 保持压力3秒...
00:08 [DEBUG] ✓ 压力保持完成

00:08 [DEBUG] 阶段4: 抬起
00:08 [DEBUG] 关闭力控模式
00:08 [DEBUG] 目标: [0.4, 0.25, 0.40, 0, 0, 0]
00:09 [DEBUG] ✓ 已抬起

[INFO] ✓ 穴位点按完成！总耗时: 9秒
```

---

### 5.3 案例2：旋转推揉节点 (TechAcupointTwistNode)

这个技法模拟用手指在穴位上旋转按压，像画圈一样。

#### 执行流程

```
穴位旋转 (Twist)
│
├─ 步骤1: 移动到穴位上方
│
├─ 步骤2: 下降并接触
│
├─ 步骤3: 旋转运动 (重点！)
│   ├─ 启动力控: 保持下压力 100N
│   ├─ 旋转方式: 围绕Z轴正弦摆动
│   ├─ 旋转角度: ±30° (左右各30度)
│   ├─ 旋转频率: 0.5Hz (每2秒一个循环)
│   ├─ 旋转周期: 10个循环
│   └─ 轨迹公式: 
│       rx(t) = rx0 + amplitude * sin(2πf*t)
│       ry(t) = ry0 + amplitude * cos(2πf*t)
│
└─ 步骤4: 抬起
```

#### 关键代码片段

```cpp
// 生成旋转轨迹点
for (int i = 0; i < num_cycles; i++) {
    // 左旋 (顺时针)
    Waypoint twist_left;
    twist_left.pose = start_pose;
    twist_left.pose[3] += twist_angle_;  // rx增加30度
    twist_left.velocity = angular_velocity_;
    twist_left.force_control_enabled = true;
    twist_left.target_force_z = pressure_;
    waypoints.push_back(twist_left);
    
    // 回中
    Waypoint twist_center;
    twist_center.pose = start_pose;
    twist_center.velocity = angular_velocity_;
    twist_center.force_control_enabled = true;
    twist_center.target_force_z = pressure_;
    waypoints.push_back(twist_center);
    
    // 右旋 (逆时针)
    Waypoint twist_right;
    twist_right.pose = start_pose;
    twist_right.pose[3] -= twist_angle_;  // rx减少30度
    twist_right.velocity = angular_velocity_;
    twist_right.force_control_enabled = true;
    twist_right.target_force_z = pressure_;
    waypoints.push_back(twist_right);
    
    // 回中
    waypoints.push_back(twist_center);
}
```

#### 实际执行例子

```
[INFO] 开始执行: 穴位旋转 (风门穴)
[DEBUG] 旋转参数: 角度=±30°, 频率=0.5Hz, 周期=10次

00:00 [DEBUG] 移动到上方并接触...
00:03 [DEBUG] 开始旋转运动

00:03 [DEBUG] 第1周期:
00:04 [DEBUG]   └─ 左旋 30° (rx: 0° → 30°)
00:05 [DEBUG]   └─ 回中 (rx: 30° → 0°)
00:06 [DEBUG]   └─ 右旋 30° (rx: 0° → -30°)
00:07 [DEBUG]   └─ 回中 (rx: -30° → 0°)

00:07 [DEBUG] 第2周期:
... (重复10次) ...

00:43 [DEBUG] 第10周期完成
00:43 [DEBUG] ✓ 旋转运动结束，抬起

[INFO] ✓ 穴位旋转完成！总耗时: 45秒
```

---

### 5.4 案例3：螺旋移动节点 (TechAcupointSpiralMoveNode)

这个技法在两个穴位之间螺旋移动，像弹簧一样。

#### 执行流程

```
螺旋移动 (Spiral)
│
├─ 输入参数:
│   ├─ 起始穴位: A (x1, y1, z1)
│   ├─ 结束穴位: B (x2, y2, z2)
│   ├─ 螺距 (pitch): 2cm
│   ├─ 半径 (radius): 1cm
│   └─ 圈数: 根据距离自动计算
│
├─ 步骤1: 移动到起点上方
│
├─ 步骤2: 下降到起点
│
├─ 步骤3: 螺旋轨迹移动 (重点！)
│   │
│   │  轨迹公式:
│   │  ─────────
│   │  t: 0 → 1 (归一化参数)
│   │  
│   │  x(t) = x1 + (x2-x1)*t + radius*cos(2π*n*t)
│   │  y(t) = y1 + (y2-y1)*t + radius*sin(2π*n*t)
│   │  z(t) = z1 + (z2-z1)*t
│   │  
│   │  其中 n = distance / pitch (圈数)
│   │
│   └─ 启动力控: 保持接触压力 80N
│
└─ 步骤4: 抬起
```

#### 可视化示例

```
起点 A (0.4, 0.2, 0.35)  →  终点 B (0.4, 0.4, 0.35)
距离 = 20cm, 螺距 = 2cm, 圈数 = 10

侧视图:                 俯视图:
  ↑ Z                     ↑ Y
  │                       │    ╱─╲
A ●═══════════● B         │  ╱     ╲
  │  ～～～螺旋～～～       │ ●───────● 
  └──────────→ Y          │  ╲     ╱
                          │    ╲─╱
                          └──────────→ X

轨迹点示例:
t=0.0: (0.400, 0.200, 0.35) - 起点
t=0.1: (0.400, 0.220, 0.35) + (0.01, 0)     螺旋偏移
t=0.2: (0.400, 0.240, 0.35) + (0, 0.01)     螺旋偏移
t=0.3: (0.400, 0.260, 0.35) + (-0.01, 0)    螺旋偏移
...
t=1.0: (0.400, 0.400, 0.35) - 终点
```

---

## 6. 数据流转过程

### 6.1 穴位数据的完整流转

```
┌─────────────────┐
│  1. 拍照        │ 
│  相机获取图像    │
└────────┬────────┘
         │ 图像数据
         ▼
┌─────────────────┐
│  2. AI识别      │
│  检测穴位位置    │
└────────┬────────┘
         │ 图片坐标 (像素)
         │ 例如: {x: 320, y: 240}
         ▼
┌─────────────────┐
│  3. 坐标转换    │
│  像素 → 3D坐标  │
└────────┬────────┘
         │ 相机坐标系
         │ 例如: {x: 0.15, y: -0.20, z: 0.45}
         ▼
┌─────────────────┐
│  4. 坐标变换    │
│  相机系 → 基坐标系│
└────────┬────────┘
         │ 机器人基坐标系
         │ 例如: {x: 0.40, y: 0.25, z: 0.35}
         ▼
┌─────────────────┐
│  5. 加载方案    │
│  匹配按摩技法    │
└────────┬────────┘
         │ Technique 结构
         │ {
         │   technique_code: "PRESS",
         │   acupoint_codes: ["GB21"],
         │   config_data: {...}
         │ }
         ▼
┌─────────────────┐
│  6. 生成行为树  │
│  创建执行计划    │
└────────┬────────┘
         │ BehaviorTree XML
         ▼
┌─────────────────┐
│  7. 执行        │
│  机械臂运动      │
└─────────────────┘
```

### 6.2 数据结构详解

#### Acupoint3DPoint (穴位三维坐标)

```cpp
struct Acupoint3DPoint
{
    std::string acupoint_code;      // 穴位代码，如 "GB21" (肩井穴)
    std::vector<double> coordinates; // 6D坐标 [x, y, z, rx, ry, rz]
    //                                 x, y, z: 位置 (米)
    //                                 rx, ry, rz: 姿态 (弧度)
};
```

**示例**：

```json
{
    "acupoint_code": "GB21",
    "coordinates": [0.40, 0.25, 0.35, 0.0, 0.0, 1.57]
}
```

解释：
- 位置: (0.40m, 0.25m, 0.35m) - 相对于机器人基座
- 姿态: (0°, 0°, 90°) - 末端执行器垂直向下

#### Technique (按摩技法)

```cpp
struct Technique
{
    std::string technique_code;              // 技法代码
    std::vector<std::string> acupoint_codes; // 穴位序列
    int repeat_count;                        // 重复次数
    nlohmann::json config_data;              // 配置参数
    std::vector<Acupoint3DPoint> acupoint_points; // 穴位坐标列表
};
```

**示例**：

```json
{
    "technique_code": "ACUPOINT_PRESS",
    "acupoint_codes": ["GB21", "SI15"],
    "repeat_count": 3,
    "config_data": {
        "pressure": 150,
        "duration": 3.0,
        "speed": 0.2
    },
    "acupoint_points": [
        {"acupoint_code": "GB21", "coordinates": [0.40, 0.25, 0.35, 0, 0, 1.57]},
        {"acupoint_code": "SI15", "coordinates": [0.42, 0.28, 0.34, 0, 0, 1.57]}
    ]
}
```

#### MassageSection (按摩段落)

```cpp
struct MassageSection
{
    int repeat_count;                    // 段落重复次数
    std::vector<Technique> techniques;   // 技法序列
};
```

**示例**：

```json
{
    "repeat_count": 2,
    "techniques": [
        {
            "technique_code": "ACUPOINT_PRESS",
            "acupoint_codes": ["GB21"],
            "repeat_count": 1,
            "config_data": {"pressure": 150}
        },
        {
            "technique_code": "ACUPOINT_TWIST",
            "acupoint_codes": ["SI15"],
            "repeat_count": 1,
            "config_data": {"angle": 30, "frequency": 0.5}
        }
    ]
}
```

解释：
```
段落重复2次:
  第1次:
    - 穴位点按 (GB21)
    - 穴位旋转 (SI15)
  第2次:
    - 穴位点按 (GB21)
    - 穴位旋转 (SI15)
```

---

## 7. 实际执行示例

### 7.1 完整的按摩流程示例

**用户场景**：张先生想要肩颈按摩

```
═══════════════════════════════════════════════════════════════
时间线:            │ 状态机状态   │ 具体操作
═══════════════════════════════════════════════════════════════
09:00:00          │ 系统开机    │ 
09:00:01          │ MsInit      │ → StHardwareCheck
                  │             │   ✓ 机械臂连接正常
                  │             │   ✓ 相机连接正常
                  │             │   ✓ 力传感器正常
09:00:05          │ MsInit      │ → StSoftwareCheck
                  │             │   ✓ ROS节点就绪
09:00:08          │ MsInit      │ → StDataInit
                  │             │   ✓ 加载配置文件
09:00:10          │ MsInit      │ → StReturnToHome
                  │             │   ✓ 机械臂归位完成
09:00:15          │ MsInit      │ → StMotionTest
                  │             │   ✓ 运动测试通过
09:00:18          │ ──转换──>   │ 
                  │ MsIdle      │ → StIdleArmHoming
                  │             │   机械臂已在Home位置
09:00:20          │ MsIdle      │ → StIdle
                  │             │   [TTS] "您好，欢迎使用"
                  │             │   等待用户选择...
───────────────────────────────────────────────────────────────
👤 用户操作: 在屏幕上选择 "肩颈按摩"
───────────────────────────────────────────────────────────────
09:01:00          │ MsIdle      │ → 用户选择肩颈部位
09:01:01          │ ──转换──>   │ (EvMassageLifeCyclePhotoTakeReady)
                  │ MsPreparation│ → StPhotoTakeReady
                  │             │   机械臂移动到拍照位置...
09:01:10          │ MsPreparation│   相机已就绪
                  │             │   [TTS] "请调整姿势，点击拍照"
───────────────────────────────────────────────────────────────
👤 用户操作: 调整好姿势，点击"拍照"按钮
───────────────────────────────────────────────────────────────
09:01:30          │ MsPreparation│ → StPhotoTakenRecognizing
                  │             │   📸 拍照中...
09:01:32          │             │   🤖 AI识别中...
09:01:35          │             │   ✓ 识别成功！检测到4个穴位:
                  │             │      • GB21 (肩井穴)
                  │             │      • SI15 (肩中俞穴)
                  │             │      • GB20 (风池穴)
                  │             │      • BL10 (天柱穴)
09:01:36          │ MsPreparation│ → StPhotoTakenRecognizeSucceed
                  │             │   加载肩颈按摩方案...
                  │             │   方案包含:
                  │             │     - 段落1: 预热 (3分钟)
                  │             │     - 段落2: 深度按摩 (10分钟)
                  │             │     - 段落3: 放松 (5分钟)
09:01:40          │             │   [TTS] "识别完成，请确认开始"
───────────────────────────────────────────────────────────────
👤 用户操作: 确认参数，点击"开始按摩"
───────────────────────────────────────────────────────────────
09:02:00          │ ──转换──>   │ (EvMassageStartReq)
                  │ MsExecution │ → StMassageWarmUp
                  │             │   [TTS] "即将开始，请放松"
09:02:05          │ MsExecution │   执行预热动作...
09:02:10          │             │   轻柔点按 GB21 (力度: 50N)
09:02:15          │             │   轻柔点按 SI15 (力度: 50N)
09:02:20          │             │   ✓ 预热完成
09:02:21          │ MsExecution │ → StMassaging
                  │             │   
                  │             │ ┌─────────────────────────┐
                  │             │ │ 开始执行行为树          │
                  │             │ └─────────────────────────┘
                  │             │   
                  │             │ ══ 段落1: 深度按摩 (重复1次) ══
09:02:22          │             │   
                  │             │ ── 技法1: 穴位点按 (GB21) ──
09:02:23          │             │   • 移动到上方... ✓
09:02:25          │             │   • 下降接触... ✓
09:02:26          │             │   • 施加压力 150N... 
                  │             │     [传感器] 120N... 145N... 150N ✓
09:02:29          │             │   • 保持3秒... ✓
09:02:32          │             │   • 抬起... ✓
09:02:33          │             │   ✓ 技法1完成
                  │             │   
                  │             │ ── 技法2: 穴位旋转 (GB21) ──
09:02:34          │             │   • 移动并接触... ✓
09:02:36          │             │   • 开始旋转 (±30°, 10周期)
09:02:37          │             │     第1周期: 左30° → 中 → 右30° → 中
09:02:41          │             │     第2周期: ...
                  │             │     ...
09:03:16          │             │     第10周期完成
09:03:17          │             │   • 抬起... ✓
09:03:18          │             │   ✓ 技法2完成
                  │             │   
                  │             │ ── 技法3: 螺旋移动 (GB21→SI15) ──
09:03:19          │             │   • 移动到起点上方... ✓
09:03:21          │             │   • 下降到起点... ✓
09:03:22          │             │   • 开始螺旋移动 (10cm, 5圈)
09:03:22          │             │     t=0%: 起点 GB21
09:03:24          │             │     t=20%: 螺旋中...
09:03:26          │             │     t=40%: 螺旋中...
09:03:28          │             │     t=60%: 螺旋中...
09:03:30          │             │     t=80%: 螺旋中...
09:03:32          │             │     t=100%: 终点 SI15 ✓
09:03:33          │             │   • 抬起... ✓
09:03:34          │             │   ✓ 技法3完成
                  │             │   
                  │             │ ── 技法4: 环形按压 (SI15) ──
09:03:35          │             │   • 移动并接触... ✓
09:03:37          │             │   • 环形轨迹 (半径2cm, 3圈)
09:03:38          │             │     第1圈: 0° → 90° → 180° → 270° → 360°
09:03:42          │             │     第2圈: ...
09:03:46          │             │     第3圈: ...
09:03:50          │             │   ✓ 技法4完成
───────────────────────────────────────────────────────────────
👤 用户操作: 感觉力度不够，点击"暂停"按钮
───────────────────────────────────────────────────────────────
09:05:00          │ MsExecution │ (EvMassagePauseReq)
09:05:01          │ MsExecution │ → StPaused
                  │             │   • 立即停止当前动作
                  │             │   • 机械臂保持位置
                  │             │   [TTS] "按摩已暂停"
                  │             │   等待用户操作...
───────────────────────────────────────────────────────────────
👤 用户操作: 在界面上调整力度 150N → 180N，点击"继续"
───────────────────────────────────────────────────────────────
09:05:30          │ MsExecution │ (EvMassageResumeReq)
09:05:31          │ MsExecution │ → StMassaging
                  │             │   • 更新力度参数: 180N
                  │             │   • 继续执行剩余技法...
09:05:32          │             │   
                  │             │ ── 技法5: 穴位点按 (GB20) ──
09:05:33          │             │   • 施加压力 180N (已更新) ... ✓
09:05:38          │             │   ✓ 技法5完成
                  │             │   
                  │             │   ... 继续执行 ...
                  │             │   
09:17:00          │             │ ══ 段落3: 放松完成 ══
09:17:01          │             │   
09:17:02          │             │   ✓ 所有按摩技法执行完毕！
09:17:03          │ MsExecution │ → StMassageSucceed
                  │             │   • 机械臂归位中...
09:17:10          │             │   • 保存按摩记录到数据库
                  │             │   • 记录内容:
                  │             │       - 用户ID: user_12345
                  │             │       - 部位: 肩颈
                  │             │       - 持续时间: 15分钟
                  │             │       - 穴位: GB21, SI15, GB20, BL10
                  │             │       - 技法数: 12
                  │             │   [TTS] "按摩完成，感谢使用！"
09:17:15          │ ──转换──>   │
                  │ MsIdle      │ → StIdleArmHoming
                  │             │   等待下一位用户...
═══════════════════════════════════════════════════════════════
```

---

## 8. 常见问题解答

### Q1: 为什么要用状态机？直接写if-else不行吗？

**答案**：

想象一下，如果用if-else:

```cpp
// ❌ 不好的做法 - 意大利面代码
void robot_main_loop() {
    if (is_init) {
        if (hardware_ok) {
            if (software_ok) {
                if (data_loaded) {
                    is_init = false;
                    is_idle = true;
                }
            }
        }
    } else if (is_idle) {
        if (user_start_request) {
            if (photo_taken) {
                if (recognize_ok) {
                    is_idle = false;
                    is_executing = true;
                }
            }
        }
    } else if (is_executing) {
        if (emergency_stop) {
            // 错误处理
        } else if (user_pause) {
            // 暂停处理
        } else if (massage_done) {
            // 完成处理
        }
        // ... 更多if嵌套
    }
    // 😱 代码难以维护！
}
```

**使用状态机的好处**：

```cpp
// ✓ 好的做法 - 清晰的状态机
struct MsInit : State {
    // 只关心初始化相关的逻辑
};

struct MsIdle : State {
    // 只关心待机相关的逻辑
};

struct MsExecution : State {
    // 只关心执行相关的逻辑
};

// 每个状态职责清晰，易于维护和扩展！
```

### Q2: 正交线(Orthogonal)是什么意思？

**答案**：

正交线让多个任务**并行执行**，互不干扰。

**例子**：

```
不使用正交线:
  机械臂移动  ──> 检测急停 ──> 更新UI ──> 记录日志
  (必须等上一个任务完成)

使用正交线:
  正交线1: 机械臂移动  ────────────────────>
  正交线2: 检测急停    ────────────────────>
  正交线3: 更新UI      ────────────────────>
  正交线4: 记录日志    ────────────────────>
  (所有任务同时进行！)
```

**实际应用**：

```cpp
// OrMonitor 正交线持续监听急停信号
// 即使在 MsExecution 执行按摩时，它也在后台运行
OrMonitor::onEntry() {
    // 订阅急停话题
    emergency_sub_ = node_->create_subscription<Bool>(
        "/emergency_stop",
        [this](const Bool::SharedPtr msg) {
            if (msg->data) {
                // 立即发送急停事件！
                this->postEvent<EvEmergencyStop>();
            }
        }
    );
}
```

### Q3: 行为树和状态机有什么区别？

**答案**：

- **状态机**：管理机器人的**生命周期**（开机、待机、执行、错误）
- **行为树**：管理**具体任务的执行**（点按、旋转、移动）

**类比**：

```
状态机 = 公司部门
  ├─ 销售部 (MsIdle - 待客)
  ├─ 生产部 (MsExecution - 生产)
  └─ 维修部 (MsError - 修理)

行为树 = 生产流程
  生产一个产品:
    ├─ 准备原料
    ├─ 加工
    ├─ 组装
    └─ 质检
```

### Q4: 如果按摩过程中出错了怎么办？

**答案**：

系统有多层保护机制：

```
1. 实时监控层 (OrMonitor, OrRiskManagement)
   └─> 持续检测: 急停、碰撞、压力异常

2. 事件触发层
   └─> 一旦检测到异常，立即发送事件:
       EvEmergencyStop, EvRiskEmergencyStop

3. 状态转换层
   └─> 任何状态都可以立即转到 MsError:
       MsInit ──┐
       MsIdle ──┤
       MsExec ──┼──> MsError (紧急处理)
       MsPrep ──┘

4. 错误处理层 (MsError)
   └─> 停止所有运动
   └─> 机械臂安全停止
   └─> 记录错误日志
   └─> 等待人工介入
```

**示例**：

```
正常执行:
  [INFO] 执行技法3...
  [INFO] 施加压力 150N...
  
异常发生:
  [WARN] 压力传感器异常: 350N (超过阈值 300N)
  [ERROR] 触发事件: EvRiskEmergencyStop
  [ERROR] 状态转换: MsExecution → MsError
  [ACTION] 立即抬起机械臂
  [ACTION] 停止所有运动
  [TTS] "检测到压力异常，按摩已停止"
  [INFO] 系统进入安全模式，等待检查...
```

### Q5: 如何添加一个新的按摩技法？

**答案 (分步骤说明)**：

**步骤1：创建技法节点头文件**

```cpp
// include/bt_nodes/tech_my_new_technique_node.hpp
#pragma once

#include <behaviortree_cpp_v3/action_node.h>

class TechMyNewTechniqueNode : public BT::StatefulActionNode
{
public:
    TechMyNewTechniqueNode(const std::string& name, 
                           const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    // 添加你的成员变量...
};
```

**步骤2：实现技法逻辑**

```cpp
// src/bt_nodes/tech_my_new_technique_node.cpp
BT::NodeStatus TechMyNewTechniqueNode::onStart()
{
    // 1. 获取输入参数
    Acupoint3DPointList acupoints;
    getInput("acupoint_points", acupoints);
    
    // 2. 创建轨迹
    std::vector<Waypoint> waypoints;
    // ... 添加你的轨迹点
    
    // 3. 发送Action请求
    action_client_->async_send_goal(goal_msg);
    
    return BT::NodeStatus::RUNNING;
}
```

**步骤3：注册到行为树工厂**

```cpp
// 在状态机初始化时注册
factory.registerNodeType<TechMyNewTechniqueNode>("MyNewTechnique");
```

**步骤4：在数据库中添加技法配置**

```json
{
    "technique_code": "MY_NEW_TECH",
    "name": "我的新技法",
    "description": "这是一个新技法",
    "default_params": {
        "speed": 0.2,
        "pressure": 150
    }
}
```

完成！现在你的新技法可以在按摩方案中使用了。

---

## 📚 总结

### 核心要点回顾

1. **状态机是机器人的生命周期管理器**
   - 5个主状态：Init → Idle → Preparation → Execution → (Error)
   - 每个状态有明确的职责和转换规则

2. **正交线实现并行任务管理**
   - 10个正交线各司其职
   - 机械臂控制、安全监控、用户服务等可同时运行

3. **行为树执行具体按摩任务**
   - 技法节点实现各种按摩手法
   - 段落和重复次数控制按摩流程

4. **数据从图像到运动的完整流转**
   - 拍照 → 识别 → 坐标转换 → 方案加载 → 行为树生成 → 执行

5. **多层安全保护机制**
   - 实时监控 + 事件触发 + 紧急停止 + 错误处理

### 学习路径建议

```
初学者 → 理解状态机基本概念
         └─> 了解5大主状态的作用和转换

进阶   → 深入研究行为树节点
         └─> 学习各种技法的实现原理

高级   → 掌握正交线和并发控制
         └─> 理解系统整体架构和数据流

专家   → 开发新技法和优化算法
         └─> 贡献新功能到系统中
```

---

**[文档结束 - 第1部分]**

由于内容较多，这是第一部分文档。如果您需要，我可以继续创建：
- 第2部分：深入每个子状态的详细实现
- 第3部分：通信接口和服务详解  
- 第4部分：实战调试和问题排查指南

请告诉我您是否需要继续创建后续部分！
