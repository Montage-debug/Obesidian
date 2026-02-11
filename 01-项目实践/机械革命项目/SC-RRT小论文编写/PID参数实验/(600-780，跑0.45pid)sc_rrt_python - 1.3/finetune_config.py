"""
微调配置：保证100%放置 + 高占用率
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.environment import EnvironmentConfig
import numpy as np

print('='*70)
print('【微调配置：保证100%放置 + 高占用率】')
print('='*70)

# 测试多组配置
configs_2d = [
    (23, 39, 225),
    (22, 38, 225),
    (24, 40, 215),
    (23, 39, 220),
]

print('\n【二维空间测试】')
print('%-20s %-15s %-10s' % ('配置', '放置', '占用率'))
print('-'*50)

best_2d = None
best_score = 0
for r_min, r_max, num in configs_2d:
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=num,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=15,
        clearance=18,
        seed=301
    )
    
    placed = len(env['obstacles'])
    rate = placed/num*100
    radii = env['obstacles'][:, 2]
    area = np.sum(np.pi * radii**2)
    occ = area / (1500*1500) * 100
    
    score = 0
    if rate >= 99.5:
        score = 100 + occ
    elif rate >= 95:
        score = 50 + occ
    
    config = f'R{r_min}-{r_max} N{num}'
    placed_str = f'{placed}/{num} ({rate:.1f}%)'
    print('%-20s %-15s %.2f%%' % (config, placed_str, occ))
    
    if score > best_score:
        best_score = score
        best_2d = (r_min, r_max, num, placed, occ)

print(f'\n✓ 推荐二维: R{best_2d[0]}-{best_2d[1]} N{best_2d[2]} → {best_2d[3]}/{best_2d[2]}, {best_2d[4]:.2f}%')

# 测试三维
configs_3d = [
    (50, 76, 400),
    (48, 74, 400),
    (52, 78, 390),
    (50, 76, 395),
]

print('\n【三维空间测试】')
print('%-20s %-15s %-10s' % ('配置', '放置', '占用率'))
print('-'*50)

best_3d = None
best_score = 0
for r_min, r_max, num in configs_3d:
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=num,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=17,
        clearance=21,
        seed=302
    )
    
    placed = len(env['obstacles'])
    rate = placed/num*100
    radii = env['obstacles'][:, 3]
    volume = np.sum(4/3 * np.pi * radii**3)
    occ = volume / (1500**3) * 100
    
    score = 0
    if rate >= 99.5:
        score = 100 + occ
    elif rate >= 95:
        score = 50 + occ
    
    config = f'R{r_min}-{r_max} N{num}'
    placed_str = f'{placed}/{num} ({rate:.1f}%)'
    print('%-20s %-15s %.2f%%' % (config, placed_str, occ))
    
    if score > best_score:
        best_score = score
        best_3d = (r_min, r_max, num, placed, occ)

print(f'\n✓ 推荐三维: R{best_3d[0]}-{best_3d[1]} N{best_3d[2]} → {best_3d[3]}/{best_3d[2]}, {best_3d[4]:.2f}%')

print('\n' + '='*70)
print('【最终配置建议】')
print('='*70)
print(f'二维: radius_range=({best_2d[0]}, {best_2d[1]}), num_obstacles={best_2d[2]}')
print(f'      放置率{best_2d[3]/best_2d[2]*100:.1f}%, 占用率{best_2d[4]:.2f}%, 迭代600')
print(f'三维: radius_range=({best_3d[0]}, {best_3d[1]}), num_obstacles={best_3d[2]}')
print(f'      放置率{best_3d[3]/best_3d[2]*100:.1f}%, 占用率{best_3d[4]:.2f}%, 迭代700')
print('='*70)
