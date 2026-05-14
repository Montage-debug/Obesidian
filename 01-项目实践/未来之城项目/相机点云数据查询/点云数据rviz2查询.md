```c
1、移至对应目录
cd ~/WorkSpace/wlzc_massage_robot_ws

2、编译节点
colcon build --packages-select robot_interfaces robot_ros_description depth_camera
source install/setup.bash

3、启动相机驱动（通过 launch 文件）
ros2 launch depth_camera camera_manager.launch.py
# 拍照
# 开启相机流
ros2 service call /camera/switch_camera robot_interfaces/srv/CameraSwitch "{enable: true}"
# 拍照并保存数据快照（这会触发点云计算和保存）
ros2 service call /camera/save_data robot_interfaces/srv/CameraSaveData "{}"
# 确认点云话题发布状态
ros2 topic echo /camera/pointcloud --no-arr | head -20
# 或查看频率
ros2 topic hz /camera/pointcloud

4、启动robot_ros_description节点
ros2 launch robot_ros_description aubo_viewer.launch.py

5、运行对应程序发送对应坐标点位标定至三维点云
ros2 run depth_camera view_point_test
# 步骤
# 1
base_link
# 2
输入对应点位[]
# 3
show-----显示
# 4
quit-----退出
```

