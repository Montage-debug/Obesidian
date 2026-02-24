"""
碰撞检测和几何工具模块
Collision Detection and Geometry Utilities
"""

import numpy as np
from typing import Optional


def is_collision_free(
    point1: np.ndarray,
    point2: np.ndarray,
    obstacles: np.ndarray,
    dim: int
) -> bool:
    """
    检查路径是否无碰撞
    
    Args:
        point1: 起点 (dim,)
        point2: 终点 (dim,)
        obstacles: 障碍物数组，2D: (N, 3), 3D: (N, 4)
        dim: 空间维度
        
    Returns:
        True表示无碰撞，False表示有碰撞
    """
    # 确保是1D数组
    point1 = np.asarray(point1).flatten()
    point2 = np.asarray(point2).flatten()
    
    # 如果没有障碍物，直接返回无碰撞
    if obstacles is None or len(obstacles) == 0:
        return True
    
    # 计算路径长度和检查点数
    path_length = np.linalg.norm(point2 - point1)
    
    # 动态调整采样密度
    if dim == 2:
        min_radius = np.min(obstacles[:, 2])
    else:
        min_radius = np.min(obstacles[:, 3])
    
    check_step = max(0.5, min_radius * 0.5)
    num_checks = int(np.ceil(path_length / check_step)) + 1
    
    # 沿路径检查碰撞
    for i in range(num_checks + 1):
        t = i / num_checks
        check_point = point1 + t * (point2 - point1)
        
        # 检查与所有障碍物的碰撞
        for obs in obstacles:
            if dim == 2:
                center = obs[:2]
                radius = obs[2]
            else:  # 3D
                center = obs[:3]
                radius = obs[3]
            
            # 距离小于半径则碰撞
            if np.linalg.norm(check_point - center) < radius:
                return False
    
    return True


def steer_point(
    from_point: np.ndarray,
    to_point: np.ndarray,
    step_size: float
) -> np.ndarray:
    """
    从from_point向to_point方向移动step_size距离
    
    Args:
        from_point: 起点
        to_point: 目标点
        step_size: 步长
        
    Returns:
        新点坐标
    """
    direction = to_point - from_point
    distance = np.linalg.norm(direction)
    
    if distance <= step_size:
        return to_point
    else:
        return from_point + (direction / distance) * step_size


def sample_point(bounds: np.ndarray, dim: int) -> np.ndarray:
    """
    在边界内随机采样点
    
    Args:
        bounds: 边界 [xmin, xmax, ymin, ymax, ...]
        dim: 维度
        
    Returns:
        采样点
    """
    sample = np.zeros(dim)
    for i in range(dim):
        sample[i] = bounds[2*i] + np.random.rand() * (bounds[2*i+1] - bounds[2*i])
    return sample


def sample_in_ellipsoid(
    focus1: np.ndarray,
    focus2: np.ndarray,
    c_best: float,
    dim: int
) -> Optional[np.ndarray]:
    """
    在椭球体内采样（用于Informed RRT）
    
    Args:
        focus1: 椭球焦点1（起点或交汇点）
        focus2: 椭球焦点2（终点或交汇点）
        c_best: 当前最优路径长度
        dim: 维度
        
    Returns:
        采样点，如果采样失败返回None
    """
    c_min = np.linalg.norm(focus2 - focus1)
    
    if c_best < c_min or c_best == np.inf:
        return None
    
    # 计算椭球参数
    center = (focus1 + focus2) / 2
    a = c_best / 2  # 长半轴
    r = np.sqrt(c_best**2 - c_min**2) / 2  # 短半轴
    
    # 在单位球内采样
    u = np.random.randn(dim)
    u = u / np.linalg.norm(u) * (np.random.rand() ** (1.0 / dim))
    
    # 缩放到椭球
    x_ball = np.zeros(dim)
    x_ball[0] = a * u[0]
    for i in range(1, dim):
        x_ball[i] = r * u[i]
    
    # 旋转矩阵
    if c_min > 0:
        direction = (focus2 - focus1) / c_min
        # 构建旋转矩阵（简化版，仅适用于2D/3D）
        if dim == 2:
            angle = np.arctan2(direction[1], direction[0])
            rotation = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
            ])
        else:  # 3D
            # 使用Rodriguez旋转公式
            z_axis = np.array([1, 0, 0])
            v = np.cross(z_axis, direction)
            s = np.linalg.norm(v)
            c = np.dot(z_axis, direction)
            
            if s < 1e-6:  # 平行
                rotation = np.eye(dim)
            else:
                vx = np.array([
                    [0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]
                ])
                rotation = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
        
        x_ellipsoid = rotation @ x_ball + center
    else:
        x_ellipsoid = x_ball + center
    
    return x_ellipsoid


def calculate_path_length(path: np.ndarray) -> float:
    """
    计算路径长度
    
    Args:
        path: 路径点数组 (N, dim)
        
    Returns:
        总路径长度
    """
    if len(path) < 2:
        return 0.0
    
    total_length = 0.0
    for i in range(len(path) - 1):
        total_length += np.linalg.norm(path[i+1] - path[i])
    
    return total_length


def calculate_path_smoothness(path: np.ndarray) -> float:
    """
    计算路径平滑度（平均转角）
    
    Args:
        path: 路径点数组 (N, dim)
        
    Returns:
        平均转角（度）
    """
    if len(path) < 3:
        return 0.0
    
    angles = []
    for i in range(1, len(path) - 1):
        v1 = path[i] - path[i-1]
        v2 = path[i+1] - path[i]
        
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm > 1e-6 and v2_norm > 1e-6:
            cos_angle = np.dot(v1, v2) / (v1_norm * v2_norm)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.arccos(cos_angle)
            angles.append(np.degrees(angle))
    
    return np.mean(angles) if angles else 0.0


def find_nearest_node(tree: np.ndarray, point: np.ndarray, dim: int) -> int:
    """
    在树中找到距离point最近的节点索引
    
    Args:
        tree: 树数组 (N, dim+4)
        point: 目标点 (dim,)
        dim: 维度
        
    Returns:
        最近节点的索引
    """
    distances = np.linalg.norm(tree[:, :dim] - point, axis=1)
    return np.argmin(distances)
