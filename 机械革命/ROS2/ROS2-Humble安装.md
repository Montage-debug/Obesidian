
## ROS2-Humble安装
```shell
wget http://fishros.com/install -O fishros && . fishros
```
## 1. Git 安装
```bash
sudo apt install git
```

## 2. 自动安装依赖
如果下载了源码文件，可以通过 rosdep 安装依赖，然后进入工作空间编译：

```bash
sudo apt install -y python3-pip
sudo pip3 install rosdepc
sudo rosdepc init
rosdepc update
# cd 到 src 上层目录（工作空间目录）
rosdepc install -i --from-path src --rosdistro humble -y
```

## 3. USB Camera
```bash
sudo apt install ros-humble-usb-cam
```

## 4. rqt
```bash
sudo apt install ros-humble-rqt
```

## 5. Gazebo
```bash
sudo apt install ros-humble-gazebo-*
```

**注意事项：**
- 为保证模型顺利加载，可将离线模型下载并放置到 `~/.gazebo/models` 路径下
- 下载链接：https://github.com/osrf/gazebo_models

**Gazebo 报错解决方案：**
如果遇到错误 `[ERROR] [gzclient-3]: process has died [pid 81421, exit code -6, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so']`，在 `~/.bashrc` 最后加上：
```bash
source /usr/share/gazebo/setup.sh
```

## 6. TF
```bash
sudo apt install ros-humble-turtle-tf2-py ros-humble-tf2-tools
sudo pip3 install transforms3d
```

## 7. XACRO
```bash
sudo apt install ros-humble-xacro
```

---
## 8.安装 C++ 编译器

```bash
在终端执行以下命令安装 GCC C++ 编译器（Ubuntu/Debian 系统默认的 C++ 编译器）：
sudo apt update
sudo apt install g++
```

如果需要更完整的开发工具链（包括 C 编译器、make 等），可以安装`build-essential`包（推荐）：


```bash
sudo apt install build-essential
```
### 验证安装

安装完成后，检查编译器是否可用：

bash

```bash
g++ --version
```

## 9.安装asio_cmake_module包
```shell
asio_cmake_module是 ROS 2 的一个官方依赖包，可通过 APT 直接安装（适用于 ROS 2 Humble 及其他版本）：
bash
sudo apt update
sudo apt install ros-humble-asio-cmake-module

```
