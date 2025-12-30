# 一、启动文件配置
```c++
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 参数定义
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # MoveIt 配置路径
    robot_name = "your_robot_name"
    moveit_config_package = f"{robot_name}_moveit_config"
    
    # 启动 MoveIt
    move_group_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare(moveit_config_package),
            'launch',
            'move_group.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_robot_description_semantic': 'true'
        }.items()
    )
    
    # RViz 节点（可选）
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(moveit_config_package),
        'config',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        move_group_launch,
        rviz_node
    ])
```