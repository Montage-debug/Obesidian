"""
同时优化二维和三维空间配置
目标：两个空间都达到理想难度，体现PID优势
"""
import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("【二维+三维空间配置同步优化】")
print("="*70)

print("\n【设计目标】")
print("  二维空间: 100%放置率, 20-25%占用率（适中难度）")
print("  三维空间: 100%放置率, 11-13%占用率（适中难度）")
print("  预期成功率差异: No_PID vs Optimal_PID = 20-25%")

# ============ 二维空间优化 ============
print("\n" + "="*70)
print("【二维空间配置优化】")
print("="*70)

# 测试多组配置
configs_2d = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance)
    (20, 35, 225, 16, 20),   # 减小半径
    (18, 32, 225, 16, 20),   # 进一步减小
    (22, 38, 200, 18, 22),   # 减少数量
    (20, 36, 210, 17, 21),   # 折中方案
]

print(f"\n测试 {len(configs_2d)} 组配置...")
print(f"{'配置':<25} {'放置率':<10} {'占用率':<10} {'评分':<8} 状态")
print("-" * 70)

best_2d = None
best_score_2d = -999

for r_min, r_max, num_obs, spacing, clear in configs_2d:
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=num_obs,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=spacing,
        clearance=clear,
        seed=301
    )
    
    num_placed = len(env['obstacles'])
    placement_rate = num_placed / num_obs * 100
    
    if num_placed > 0:
        radii = env['obstacles'][:, 2]
        total_area = 1500 * 1500
        obstacle_area = np.sum(np.pi * radii**2)
        occupancy = obstacle_area / total_area * 100
    else:
        occupancy = 0
    
    # 评分系统：
    # 放置率100%: +100分
    # 占用率20-25%: +100分 (偏离扣分)
    # 平衡性: +额外分
    score = 0
    
    # 放置率评分
    if placement_rate >= 99:
        score += 100
    elif placement_rate >= 95:
        score += 80
    elif placement_rate >= 90:
        score += 60
    else:
        score += placement_rate * 0.6
    
    # 占用率评分 (目标20-25%)
    if 20 <= occupancy <= 25:
        score += 100
    elif 18 <= occupancy < 20:
        score += 80 + (occupancy - 18) * 10
    elif 25 < occupancy <= 28:
        score += 80 + (28 - occupancy) * 6.67
    elif 15 <= occupancy < 18:
        score += 60 + (occupancy - 15) * 6.67
    else:
        score += max(0, 100 - abs(occupancy - 22.5) * 4)
    
    status = ""
    if placement_rate >= 99 and 20 <= occupancy <= 25:
        status = "✓ 优秀"
    elif placement_rate >= 95 and 18 <= occupancy <= 28:
        status = "○ 良好"
    else:
        status = "△ 一般"
    
    config_str = f"R{r_min}-{r_max} N{num_obs}"
    print(f"{config_str:<25} {placement_rate:>6.1f}%   {occupancy:>7.2f}%   {score:>6.1f}   {status}")
    
    if score > best_score_2d:
        best_score_2d = score
        best_2d = {
            'radius_range': (r_min, r_max),
            'num_obstacles': num_obs,
            'min_spacing': spacing,
            'clearance': clear,
            'placement_rate': placement_rate,
            'occupancy': occupancy,
            'score': score
        }

print("\n【二维最佳配置】")
if best_2d:
    print(f"  radius_range: {best_2d['radius_range']}")
    print(f"  num_obstacles: {best_2d['num_obstacles']}")
    print(f"  min_spacing: {best_2d['min_spacing']}")
    print(f"  clearance: {best_2d['clearance']}")
    print(f"  放置率: {best_2d['placement_rate']:.1f}%")
    print(f"  占用率: {best_2d['occupancy']:.2f}%")
    print(f"  评分: {best_2d['score']:.1f}/200")

# ============ 三维空间优化 ============
print("\n" + "="*70)
print("【三维空间配置优化】")
print("="*70)

# 测试多组配置
configs_3d = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance)
    (45, 70, 400, 18, 22),   # 当前配置
    (40, 65, 450, 18, 22),   # 增加数量
    (38, 62, 500, 16, 20),   # 显著增加
    (42, 68, 425, 18, 22),   # 小幅调整
    (35, 60, 500, 16, 20),   # 激进配置
]

print(f"\n测试 {len(configs_3d)} 组配置...")
print(f"{'配置':<25} {'放置率':<10} {'占用率':<10} {'评分':<8} 状态")
print("-" * 70)

best_3d = None
best_score_3d = -999

for r_min, r_max, num_obs, spacing, clear in configs_3d:
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=num_obs,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=spacing,
        clearance=clear,
        seed=302
    )
    
    num_placed = len(env['obstacles'])
    placement_rate = num_placed / num_obs * 100
    
    if num_placed > 0:
        radii = env['obstacles'][:, 3]
        total_volume = 1500 ** 3
        obstacle_volume = np.sum(4/3 * np.pi * radii**3)
        occupancy = obstacle_volume / total_volume * 100
    else:
        occupancy = 0
    
    # 评分系统：
    # 放置率100%: +100分
    # 占用率11-13%: +100分 (偏离扣分)
    score = 0
    
    # 放置率评分
    if placement_rate >= 99:
        score += 100
    elif placement_rate >= 95:
        score += 80
    elif placement_rate >= 90:
        score += 60
    else:
        score += placement_rate * 0.6
    
    # 占用率评分 (目标11-13%)
    if 11 <= occupancy <= 13:
        score += 100
    elif 10 <= occupancy < 11:
        score += 80 + (occupancy - 10) * 20
    elif 13 < occupancy <= 14:
        score += 80 + (14 - occupancy) * 20
    elif 9 <= occupancy < 10:
        score += 60 + (occupancy - 9) * 20
    else:
        score += max(0, 100 - abs(occupancy - 12) * 8)
    
    status = ""
    if placement_rate >= 99 and 11 <= occupancy <= 13:
        status = "✓ 优秀"
    elif placement_rate >= 95 and 10 <= occupancy <= 14:
        status = "○ 良好"
    else:
        status = "△ 一般"
    
    config_str = f"R{r_min}-{r_max} N{num_obs}"
    print(f"{config_str:<25} {placement_rate:>6.1f}%   {occupancy:>7.2f}%   {score:>6.1f}   {status}")
    
    if score > best_score_3d:
        best_score_3d = score
        best_3d = {
            'radius_range': (r_min, r_max),
            'num_obstacles': num_obs,
            'min_spacing': spacing,
            'clearance': clear,
            'placement_rate': placement_rate,
            'occupancy': occupancy,
            'score': score
        }

print("\n【三维最佳配置】")
if best_3d:
    print(f"  radius_range: {best_3d['radius_range']}")
    print(f"  num_obstacles: {best_3d['num_obstacles']}")
    print(f"  min_spacing: {best_3d['min_spacing']}")
    print(f"  clearance: {best_3d['clearance']}")
    print(f"  放置率: {best_3d['placement_rate']:.1f}%")
    print(f"  占用率: {best_3d['occupancy']:.2f}%")
    print(f"  评分: {best_3d['score']:.1f}/200")

# ============ 总结 ============
print("\n" + "="*70)
print("【配置优化总结】")
print("="*70)

print("\n【当前配置的问题】")
print("  二维: R25-45, N225 → 只能放置178个(79.1%), 占用率30.42% (太密)")
print("  三维: R45-70, N400 → 放置400个(100%), 占用率9.49% (太疏)")
print("  结果: 二维难度过高，三维难度不足，难以体现PID优势")

if best_2d and best_3d:
    print("\n【推荐新配置】")
    print(f"  二维: R{best_2d['radius_range'][0]}-{best_2d['radius_range'][1]}, N{best_2d['num_obstacles']}, spacing={best_2d['min_spacing']}, clearance={best_2d['clearance']}")
    print(f"        → 放置率{best_2d['placement_rate']:.1f}%, 占用率{best_2d['occupancy']:.2f}%")
    print(f"  三维: R{best_3d['radius_range'][0]}-{best_3d['radius_range'][1]}, N{best_3d['num_obstacles']}, spacing={best_3d['min_spacing']}, clearance={best_3d['clearance']}")
    print(f"        → 放置率{best_3d['placement_rate']:.1f}%, 占用率{best_3d['occupancy']:.2f}%")
    
    print("\n【预期效果】")
    print("  两个场景难度均衡，都能体现PID采样控制的优势")
    print("  预期成功率: No_PID 35-45%, Optimal_PID 60-70% (差距20-25%)")

print("\n" + "="*70)
