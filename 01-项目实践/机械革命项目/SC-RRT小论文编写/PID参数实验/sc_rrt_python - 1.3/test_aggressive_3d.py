"""
三维空间激进配置测试
目标：达到11-13%占用率
"""
import sys
import os

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("【三维空间激进配置测试】")
print("="*70)
print("\n目标：11-13%占用率")

# 测试更激进的配置
configs = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance)
    (42, 68, 425, 18, 22),   # 基准
    (45, 75, 425, 18, 22),   # 增大半径
    (48, 78, 425, 18, 22),   # 再增大
    (40, 70, 500, 16, 20),   # 增加数量
    (42, 72, 475, 17, 21),   # 折中
    (45, 75, 475, 17, 21),   # 综合提升
    (48, 80, 450, 17, 21),   # 大半径中数量
]

print(f"\n{'配置':<25} {'放置':<10} {'占用率':<10} {'评分':<8} 状态")
print("-" * 70)

best = None
best_score = -999

for r_min, r_max, num_obs, spacing, clear in configs:
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
    
    # 评分
    score = 0
    if placement_rate >= 99:
        score += 100
    elif placement_rate >= 95:
        score += 80
    else:
        score += placement_rate * 0.6
    
    # 占用率11-13%
    if 11 <= occupancy <= 13:
        score += 100
    elif 10 <= occupancy < 11:
        score += 80 + (occupancy - 10) * 20
    elif 13 < occupancy <= 14:
        score += 80 + (14 - occupancy) * 20
    else:
        score += max(0, 100 - abs(occupancy - 12) * 8)
    
    status = ""
    if placement_rate >= 99 and 11 <= occupancy <= 13:
        status = "✓ 优秀"
    elif placement_rate >= 95 and 10 <= occupancy <= 14:
        status = "○ 良好"
    elif placement_rate >= 95 and 9 <= occupancy < 10:
        status = "△ 可用"
    else:
        status = "✗ 不佳"
    
    config_str = f"R{r_min}-{r_max} N{num_obs}"
    placed_str = f"{num_placed}/{num_obs}"
    print(f"{config_str:<25} {placed_str:<10} {occupancy:>7.2f}%   {score:>6.1f}   {status}")
    
    if score > best_score:
        best_score = score
        best = {
            'radius_range': (r_min, r_max),
            'num_obstacles': num_obs,
            'min_spacing': spacing,
            'clearance': clear,
            'placement_rate': placement_rate,
            'placed': num_placed,
            'occupancy': occupancy,
            'score': score
        }

print("\n" + "="*70)
if best:
    print("【最佳配置】")
    print(f"  radius_range: {best['radius_range']}")
    print(f"  num_obstacles: {best['num_obstacles']}")
    print(f"  min_spacing: {best['min_spacing']}")
    print(f"  clearance: {best['clearance']}")
    print(f"  实际放置: {best['placed']}/{best['num_obstacles']} ({best['placement_rate']:.1f}%)")
    print(f"  占用率: {best['occupancy']:.2f}%")
    print(f"  评分: {best['score']:.1f}/200")
    
    if best['occupancy'] >= 11:
        print("\n✓ 达到目标占用率 (11-13%)")
    elif best['occupancy'] >= 10:
        print("\n○ 接近目标占用率")
    else:
        print("\n⚠ 未达到目标，建议进一步增大半径或数量")
        # 计算需要多少个障碍物才能达到12%
        avg_radius = (best['radius_range'][0] + best['radius_range'][1]) / 2
        avg_volume = 4/3 * np.pi * avg_radius**3
        total_volume = 1500 ** 3
        target_volume = total_volume * 0.12
        needed_num = int(target_volume / avg_volume)
        print(f"  建议: 增加到约{needed_num}个障碍物可达12%占用率")
        print(f"  或者: 将半径范围提升到约R{best['radius_range'][0]+8}-{best['radius_range'][1]+8}")

print("="*70)
