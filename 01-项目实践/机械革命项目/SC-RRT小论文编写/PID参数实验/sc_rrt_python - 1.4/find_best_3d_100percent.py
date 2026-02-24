"""
寻找三维空间100%放置率下的最高占用率配置
"""
import sys
import os

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("【三维空间：100%放置率下的最高占用率】")
print("="*70)

print("\n策略：从保守到激进，找到能100%放置的最高占用率配置")

configs = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance)
    (38, 62, 500, 16, 20),   # 7.88%
    (40, 64, 500, 16, 20),
    (42, 66, 500, 16, 20),
    (44, 68, 500, 16, 20),
    (45, 70, 500, 16, 20),
    (38, 62, 525, 15, 19),
    (40, 64, 525, 15, 19),
    (42, 66, 525, 15, 19),
    (40, 65, 475, 17, 21),
    (42, 68, 475, 17, 21),
]

print(f"\n{'配置':<25} {'放置':<12} {'占用率':<10} 状态")
print("-" * 65)

best_with_100 = None
best_occupancy = 0

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
    
    status = ""
    if placement_rate >= 100:
        status = "✓ 100%放置"
        if occupancy > best_occupancy:
            best_occupancy = occupancy
            best_with_100 = {
                'radius_range': (r_min, r_max),
                'num_obstacles': num_obs,
                'min_spacing': spacing,
                'clearance': clear,
                'placement_rate': placement_rate,
                'placed': num_placed,
                'occupancy': occupancy
            }
    elif placement_rate >= 95:
        status = "○ 95%以上"
    else:
        status = f"✗ {placement_rate:.0f}%"
    
    config_str = f"R{r_min}-{r_max} N{num_obs}"
    placed_str = f"{num_placed}/{num_obs}"
    print(f"{config_str:<25} {placed_str:<12} {occupancy:>7.2f}%   {status}")

print("\n" + "="*70)
if best_with_100:
    print("【100%放置率下的最佳配置】")
    print(f"  radius_range: {best_with_100['radius_range']}")
    print(f"  num_obstacles: {best_with_100['num_obstacles']}")
    print(f"  min_spacing: {best_with_100['min_spacing']}")
    print(f"  clearance: {best_with_100['clearance']}")
    print(f"  实际放置: {best_with_100['placed']}/{best_with_100['num_obstacles']} ({best_with_100['placement_rate']:.1f}%)")
    print(f"  占用率: {best_with_100['occupancy']:.2f}%")
    
    print("\n【评估】")
    occ = best_with_100['occupancy']
    if occ >= 10:
        print(f"  ✓ 占用率{occ:.2f}%接近目标（10%+）")
        print("  预期：三维空间难度适中，可体现PID优势")
    elif occ >= 9:
        print(f"  ○ 占用率{occ:.2f}%略低于理想（9-10%）")
        print("  预期：三维空间略简单，但考虑到三维复杂性，可接受")
    else:
        print(f"  △ 占用率{occ:.2f}%偏低（<9%）")
        print("  建议：接受现状或考虑其他方法增加难度")
else:
    print("【结果】未找到能100%放置的配置")

print("="*70)
