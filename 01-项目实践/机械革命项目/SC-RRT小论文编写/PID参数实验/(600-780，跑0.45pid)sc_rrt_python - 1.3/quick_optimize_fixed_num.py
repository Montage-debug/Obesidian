"""
快速优化：固定数量（二维225，三维400）
"""
import sys
import os
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("快速优化：二维225个 + 三维400个")
print("="*70)

# 二维：测试能放置225个的配置
print("\n[二维空间：N225]")
configs_2d = [
    (20, 35, 15, 18),
    (18, 32, 15, 18),
    (19, 34, 15, 18),
    (20, 36, 15, 18),
    (21, 37, 16, 19),
    (19, 33, 15, 18),
]

best_2d = None
for r_min, r_max, spacing, clear in configs_2d:
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=225,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=spacing,
        clearance=clear,
        seed=301
    )
    num = len(env['obstacles'])
    if num > 0:
        radii = env['obstacles'][:, 2]
        area = np.sum(np.pi * radii**2)
        occ = area / (1500*1500) * 100
        
        status = "OK" if num >= 225 else f"{num}"
        print(f"  R{r_min}-{r_max} S{spacing}: {num}/225 ({num/225*100:.1f}%), {occ:.2f}% [{status}]")
        
        if num >= 225:
            if best_2d is None or occ > best_2d['occ']:
                best_2d = {'r': (r_min, r_max), 's': spacing, 'c': clear, 'occ': occ, 'num': num}

# 三维：测试400个下的最高占用率
print("\n[三维空间：N400]")
configs_3d = [
    (45, 70, 18, 22),  # 当前
    (48, 72, 18, 22),
    (50, 75, 18, 22),
    (52, 78, 18, 22),
    (48, 73, 17, 21),
    (50, 76, 17, 21),
    (52, 80, 17, 21),
    (55, 82, 17, 21),
]

best_3d = None
for r_min, r_max, spacing, clear in configs_3d:
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=400,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=spacing,
        clearance=clear,
        seed=302
    )
    num = len(env['obstacles'])
    if num > 0:
        radii = env['obstacles'][:, 3]
        vol = np.sum(4/3 * np.pi * radii**3)
        occ = vol / (1500**3) * 100
        
        status = "OK" if num >= 400 else f"{num}"
        print(f"  R{r_min}-{r_max} S{spacing}: {num}/400 ({num/400*100:.1f}%), {occ:.2f}% [{status}]")
        
        if num >= 400:
            if best_3d is None or occ > best_3d['occ']:
                best_3d = {'r': (r_min, r_max), 's': spacing, 'c': clear, 'occ': occ, 'num': num}

print("\n" + "="*70)
print("最终推荐配置")
print("="*70)

if best_2d:
    print(f"\n二维: R{best_2d['r'][0]}-{best_2d['r'][1]}, N225, spacing={best_2d['s']}, clearance={best_2d['c']}")
    print(f"      {best_2d['num']}/225 (100%), 占用率{best_2d['occ']:.2f}%")
else:
    print("\n二维: 未找到能100%放置225个的配置")

if best_3d:
    print(f"\n三维: R{best_3d['r'][0]}-{best_3d['r'][1]}, N400, spacing={best_3d['s']}, clearance={best_3d['c']}")
    print(f"      {best_3d['num']}/400 (100%), 占用率{best_3d['occ']:.2f}%")
else:
    print("\n三维: 未找到能100%放置400个的配置")

print("\n" + "="*70)
