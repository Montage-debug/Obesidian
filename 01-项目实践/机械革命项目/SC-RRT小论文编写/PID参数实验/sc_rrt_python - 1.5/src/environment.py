"""
环境配置模块
Environment Configuration Module for SC-RRT

生成2D/3D障碍物环境，确保起点终点与障碍物无碰撞
"""

import numpy as np
from typing import Tuple, Optional, Dict, List


class EnvironmentConfig:
    """环境配置类"""
    
    DEFAULT_MIN_SPACING = 30.0  # 障碍物之间最小间距
    DEFAULT_START_GOAL_CLEARANCE = 45.0  # 起点终点与障碍物的最小间距
    MAX_PLACEMENT_ATTEMPTS = 1000  # 放置障碍物的最大尝试次数
    
    @staticmethod
    def generate_2d_environment(
        bounds: List[float],
        num_obstacles: int,
        start_point: Optional[np.ndarray] = None,
        goal_point: Optional[np.ndarray] = None,
        radius_range: Tuple[float, float] = (10.0, 25.0),
        min_spacing: float = 30.0,
        clearance: float = 45.0,
        seed: Optional[int] = None
    ) -> Dict:
        """
        生成2D环境配置
        
        Args:
            bounds: [xmin, xmax, ymin, ymax] - 环境边界
            num_obstacles: 障碍物数量
            start_point: 起点 [x, y]，默认为左下角
            goal_point: 终点 [x, y]，默认为右上角
            radius_range: 半径范围 (min, max)
            min_spacing: 障碍物之间最小间距
            clearance: 起点终点与障碍物的最小间距
            seed: 随机种子
            
        Returns:
            env: 环境字典
        """
        if seed is not None:
            np.random.seed(seed)
        
        xmin, xmax, ymin, ymax = bounds
        
        # 设置默认起点和终点
        if start_point is None:
            start_point = np.array([xmin + 100, ymin + 100])
        if goal_point is None:
            goal_point = np.array([xmax - 100, ymax - 100])
        
        # 生成障碍物
        obstacles = []
        attempts = 0
        
        while len(obstacles) < num_obstacles and attempts < EnvironmentConfig.MAX_PLACEMENT_ATTEMPTS:
            attempts += 1
            
            # 随机生成障碍物位置和半径
            x = xmin + np.random.rand() * (xmax - xmin)
            y = ymin + np.random.rand() * (ymax - ymin)
            radius = radius_range[0] + np.random.rand() * (radius_range[1] - radius_range[0])
            
            new_obstacle = np.array([x, y, radius])
            
            # 检查与起点和终点的距离
            dist_to_start = np.linalg.norm(new_obstacle[:2] - start_point)
            dist_to_goal = np.linalg.norm(new_obstacle[:2] - goal_point)
            
            if dist_to_start < clearance + radius or dist_to_goal < clearance + radius:
                continue
            
            # 检查与其他障碍物的距离
            valid = True
            for obs in obstacles:
                dist = np.linalg.norm(new_obstacle[:2] - obs[:2])
                if dist < min_spacing + radius + obs[2]:
                    valid = False
                    break
            
            if valid:
                obstacles.append(new_obstacle)
        
        if len(obstacles) < num_obstacles:
            print(f"警告: 只成功放置了 {len(obstacles)}/{num_obstacles} 个障碍物")
        
        return {
            'dimension': 2,
            'bounds': bounds,
            'start': start_point,
            'goal': goal_point,
            'obstacles': np.array(obstacles) if obstacles else np.empty((0, 3)),
            'num': len(obstacles)
        }
    
    @staticmethod
    def generate_3d_environment(
        bounds: List[float],
        num_obstacles: int,
        start_point: Optional[np.ndarray] = None,
        goal_point: Optional[np.ndarray] = None,
        radius_range: Tuple[float, float] = (5.0, 15.0),
        min_spacing: float = 20.0,
        clearance: float = 30.0,
        seed: Optional[int] = None
    ) -> Dict:
        """
        生成3D环境配置
        
        Args:
            bounds: [xmin, xmax, ymin, ymax, zmin, zmax] - 环境边界
            num_obstacles: 障碍物数量
            start_point: 起点 [x, y, z]
            goal_point: 终点 [x, y, z]
            radius_range: 半径范围 (min, max)
            min_spacing: 障碍物之间最小间距
            clearance: 起点终点与障碍物的最小间距
            seed: 随机种子
            
        Returns:
            env: 环境字典
        """
        if seed is not None:
            np.random.seed(seed)
        
        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        
        # 设置默认起点和终点
        if start_point is None:
            start_point = np.array([xmin + 50, ymin + 50, zmin + 50])
        if goal_point is None:
            goal_point = np.array([xmax - 50, ymax - 50, zmax - 50])
        
        # 生成障碍物
        obstacles = []
        attempts = 0
        
        while len(obstacles) < num_obstacles and attempts < EnvironmentConfig.MAX_PLACEMENT_ATTEMPTS:
            attempts += 1
            
            # 随机生成障碍物位置和半径
            x = xmin + np.random.rand() * (xmax - xmin)
            y = ymin + np.random.rand() * (ymax - ymin)
            z = zmin + np.random.rand() * (zmax - zmin)
            radius = radius_range[0] + np.random.rand() * (radius_range[1] - radius_range[0])
            
            new_obstacle = np.array([x, y, z, radius])
            
            # 检查与起点和终点的距离
            dist_to_start = np.linalg.norm(new_obstacle[:3] - start_point)
            dist_to_goal = np.linalg.norm(new_obstacle[:3] - goal_point)
            
            if dist_to_start < clearance + radius or dist_to_goal < clearance + radius:
                continue
            
            # 检查与其他障碍物的距离
            valid = True
            for obs in obstacles:
                dist = np.linalg.norm(new_obstacle[:3] - obs[:3])
                if dist < min_spacing + radius + obs[3]:
                    valid = False
                    break
            
            if valid:
                obstacles.append(new_obstacle)
        
        if len(obstacles) < num_obstacles:
            print(f"警告: 只成功放置了 {len(obstacles)}/{num_obstacles} 个障碍物")
        
        return {
            'dimension': 3,
            'bounds': bounds,
            'start': start_point,
            'goal': goal_point,
            'obstacles': np.array(obstacles) if obstacles else np.empty((0, 4)),
            'num': len(obstacles)
        }
