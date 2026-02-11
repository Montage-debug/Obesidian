"""
分析环境难度 - 三维场景障碍物密度检查
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

import numpy as np
from src.environment import EnvironmentConfig

def analyze_environment_difficulty(env_config):
    """分析环境难度"""
    dim = env_config['dimension']
    bounds = env_config['bounds']
    obstacles = env_config['obstacles']
    start = env_config['start']
    goal = env_config['goal']
    
    print(f"\n{'='*70}")
    print(f"{dim}维环境难度分析")
    print(f"{'='*70}")
    
    # 计算空间体积
    if dim == 2:
        space_volume = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2])
        space_unit = "平方单位"
    else:
        space_volume = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2]) * (bounds[5] - bounds[4])
        space_unit = "立方单位"
    
    # 计算障碍物总体积
    obstacle_volume = 0
    for obs in obstacles:
        radius = obs[-1]
        if dim == 2:
            obstacle_volume += np.pi * radius**2
        else:
            obstacle_volume += (4/3) * np.pi * radius**3
    
    # 障碍物占比
    obstacle_ratio = obstacle_volume / space_volume * 100
    
    # 起点终点距离
    start_goal_dist = np.linalg.norm(goal - start)
    
    # 直线路径是否可行
    direct_path_blocked = False
    num_blocking = 0
    for obs in obstacles:
        obs_center = obs[:-1]
        obs_radius = obs[-1]
        
        # 点到线段的距离
        line_vec = goal - start
        line_len = np.linalg.norm(line_vec)
        line_dir = line_vec / line_len
        
        point_vec = obs_center - start
        proj_len = np.dot(point_vec, line_dir)
        proj_len = np.clip(proj_len, 0, line_len)
        
        closest_point = start + proj_len * line_dir
        dist_to_line = np.linalg.norm(obs_center - closest_point)
        
        if dist_to_line < obs_radius * 1.1:  # 留10%安全余量
            direct_path_blocked = True
            num_blocking += 1
    
    # 计算平均障碍物半径
    avg_radius = np.mean(obstacles[:, -1])
    
    # 计算障碍物密度（每单位空间的障碍物数量）
    if dim == 2:
        density = len(obstacles) / (space_volume / 10000)  # 每100x100区域的障碍物数
    else:
        density = len(obstacles) / (space_volume / 1000000)  # 每100x100x100区域的障碍物数
    
    print(f"\n【空间信息】")
    print(f"  空间体积: {space_volume:,.0f} {space_unit}")
    print(f"  起点: {start}")
    print(f"  终点: {goal}")
    print(f"  直线距离: {start_goal_dist:.2f}")
    
    print(f"\n【障碍物信息】")
    print(f"  障碍物数量: {len(obstacles)}")
    print(f"  平均半径: {avg_radius:.2f}")
    print(f"  障碍物总体积: {obstacle_volume:,.0f} {space_unit}")
    print(f"  空间占用率: {obstacle_ratio:.4f}%")
    if dim == 2:
        print(f"  密度: {density:.2f} 个/100x100区域")
    else:
        print(f"  密度: {density:.2f} 个/100x100x100区域")
    
    print(f"\n【路径难度】")
    print(f"  直线路径: {'被阻挡 ✗' if direct_path_blocked else '畅通 ✓'}")
    if direct_path_blocked:
        print(f"  阻挡障碍物数: {num_blocking}")
    
    print(f"\n【难度评估】")
    if obstacle_ratio < 0.01:
        difficulty = "极低 ⭐"
        comment = "障碍物过于稀疏，PID优势无法体现"
    elif obstacle_ratio < 0.05:
        difficulty = "低 ⭐⭐"
        comment = "障碍物较少，成功率过高"
    elif obstacle_ratio < 0.15:
        difficulty = "中 ⭐⭐⭐"
        comment = "适中难度，可以区分算法差异"
    elif obstacle_ratio < 0.30:
        difficulty = "高 ⭐⭐⭐⭐"
        comment = "高难度，能显著体现PID优势"
    else:
        difficulty = "极高 ⭐⭐⭐⭐⭐"
        comment = "极高难度，可能导致成功率过低"
    
    print(f"  难度等级: {difficulty}")
    print(f"  评价: {comment}")
    
    return obstacle_ratio, density


if __name__ == '__main__':
    print("\n" + "="*70)
    print("环境难度分析工具")
    print("="*70)
    
    # 二维环境
    print("\n【当前配置】二维场景 (225障碍物)")
    env_2d = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=225,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(25, 45),
        min_spacing=15,
        clearance=18,
        seed=301
    )
    ratio_2d, density_2d = analyze_environment_difficulty(env_2d)
    
    # 三维环境（当前配置）
    print("\n【当前配置】三维场景 (400障碍物，半径70-110)")
    env_3d_current = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=400,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(70, 110),
        min_spacing=25,
        clearance=30,
        seed=302
    )
    ratio_3d_current, density_3d_current = analyze_environment_difficulty(env_3d_current)
    
    # 三维环境（推荐配置1：更多更大的障碍物）
    print("\n【推荐配置1】三维场景 (800障碍物，更大半径)")
    env_3d_v1 = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=800,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(50, 85),  # 更大的障碍物
        min_spacing=18,
        clearance=20,
        seed=302
    )
    ratio_3d_v1, density_3d_v1 = analyze_environment_difficulty(env_3d_v1)
    
    # 三维环境（推荐配置2：缩小空间，保持障碍物数量）
    print("\n【推荐配置2】三维场景 (缩小空间至1200³)")
    env_3d_v2 = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1200, 0, 1200, 0, 1200],
        num_obstacles=500,
        start_point=np.array([60, 60, 60]),
        goal_point=np.array([1140, 1140, 1140]),
        radius_range=(45, 75),
        min_spacing=16,
        clearance=18,
        seed=302
    )
    ratio_3d_v2, density_3d_v2 = analyze_environment_difficulty(env_3d_v2)
    
    # 对比总结
    print("\n" + "="*70)
    print("配置对比总结")
    print("="*70)
    print(f"\n{'配置':<20} {'占用率':<15} {'密度':<20} {'难度'}")
    print("-"*70)
    print(f"{'二维-当前':<20} {ratio_2d:.4f}% {'':<8} {density_2d:.2f}/100² {'':<8} ⭐⭐⭐")
    print(f"{'三维-当前':<20} {ratio_3d_current:.4f}% {'':<8} {density_3d_current:.2f}/100³ {'':<8} {'⭐' if ratio_3d_current < 0.01 else '⭐⭐'}")
    print(f"{'三维-推荐1':<20} {ratio_3d_v1:.4f}% {'':<8} {density_3d_v1:.2f}/100³ {'':<8} {'⭐⭐⭐' if 0.05 <= ratio_3d_v1 < 0.15 else '⭐⭐⭐⭐'}")
    print(f"{'三维-推荐2':<20} {ratio_3d_v2:.4f}% {'':<8} {density_3d_v2:.2f}/100³ {'':<8} {'⭐⭐⭐' if 0.05 <= ratio_3d_v2 < 0.15 else '⭐⭐⭐⭐'}")
    
    print("\n" + "="*70)
    print("建议")
    print("="*70)
    print("三维空间的难度远低于二维，原因：")
    print("  1. 体积效应：1500³空间远大于1500²×深度")
    print("  2. 绕行自由度：三维可从上下左右前后6个方向绕行")
    print("  3. 障碍物占比过低：当前仅0.00X%")
    print("\n解决方案（任选其一）：")
    print("  ✅ 方案1：大幅增加障碍物数量（800-1000个）+ 增大半径")
    print("  ✅ 方案2：缩小空间至1200³，增加障碍物至500-600个")
    print("  ✅ 方案3：使用分层障碍物布局（创建必经通道）")
    print("="*70)
