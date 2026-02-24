"""
诊断二维空间配置问题
"""
import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

# 生成二维环境
env = EnvironmentConfig.generate_2d_environment(
    bounds=[0, 1500, 0, 1500],
    num_obstacles=225,
    start_point=np.array([75, 75]),
    goal_point=np.array([1425, 1425]),
    radius_range=(25, 45),
    min_spacing=15,
    clearance=18,
    seed=301
)

obstacles = env['obstacles']
num_placed = len(obstacles)

print("="*70)
print("【二维空间配置诊断】")
print("="*70)

print(f"\n目标障碍物数量: 225")
print(f"实际放置数量: {num_placed}")
print(f"放置率: {num_placed/225*100:.1f}%")

if num_placed > 0:
    radii = obstacles[:, 2]
    print(f"\n障碍物半径范围: [{radii.min():.1f}, {radii.max():.1f}]")
    print(f"障碍物平均半径: {radii.mean():.1f}")
    print(f"障碍物半径标准差: {radii.std():.1f}")
    
    # 计算空间占用率
    total_area = 1500 * 1500
    obstacle_area = np.sum(np.pi * radii**2)
    occupancy = obstacle_area / total_area * 100
    
    print(f"\n总空间: {total_area:.0f}")
    print(f"障碍物占用: {obstacle_area:.0f}")
    print(f"空间占用率: {occupancy:.2f}%")
    
    print(f"\n配置参数:")
    print(f"  radius_range: (25, 45)")
    print(f"  min_spacing: 15")
    print(f"  clearance: 18")
    
    # 分析问题
    print(f"\n【问题分析】")
    avg_radius = radii.mean()
    avg_obstacle_spacing = avg_radius * 2 + 15
    print(f"平均障碍物半径: {avg_radius:.1f}")
    print(f"平均障碍物直径: {avg_radius*2:.1f}")
    print(f"平均障碍物间距: {avg_obstacle_spacing:.1f}")
    print(f"有效放置单元大小: {avg_obstacle_spacing:.1f} × {avg_obstacle_spacing:.1f}")
    
    # 理论最大放置数量
    grid_size = avg_obstacle_spacing
    max_x = int(1500 / grid_size)
    max_y = int(1500 / grid_size)
    theoretical_max = max_x * max_y
    print(f"\n理论网格: {max_x} × {max_y} = {theoretical_max} 个")
    print(f"考虑随机性，实际可放置约: {int(theoretical_max * 0.7)} 个")
    
    print(f"\n【推荐配置】")
    # 要放置225个障碍物
    target = 225
    # 考虑70%的放置效率，需要网格大小
    grid_needed = int(np.sqrt(target / 0.7))  # ~18
    ideal_spacing = 1500 / grid_needed  # ~83
    # spacing = 2*R + min_spacing
    # 假设min_spacing=18, 则2*R = 65, R=32.5
    ideal_avg_radius = (ideal_spacing - 18) / 2
    ideal_min_radius = ideal_avg_radius - 10
    ideal_max_radius = ideal_avg_radius + 10
    
    print(f"目标: 225个障碍物")
    print(f"建议平均半径: {ideal_avg_radius:.1f}")
    print(f"建议半径范围: ({ideal_min_radius:.0f}, {ideal_max_radius:.0f})")
    print(f"建议min_spacing: 18 (保持)")
    print(f"建议clearance: 22 (保持)")
    
    # 测试推荐配置
    print(f"\n【验证推荐配置】")
    test_env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=225,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(int(ideal_min_radius), int(ideal_max_radius)),
        min_spacing=18,
        clearance=22,
        seed=301
    )
    
    test_num = len(test_env['obstacles'])
    test_radii = test_env['obstacles'][:, 2]
    test_area = np.sum(np.pi * test_radii**2)
    test_occupancy = test_area / total_area * 100
    
    print(f"实际放置: {test_num}/225 ({test_num/225*100:.1f}%)")
    print(f"空间占用率: {test_occupancy:.2f}%")

print("\n" + "="*70)
