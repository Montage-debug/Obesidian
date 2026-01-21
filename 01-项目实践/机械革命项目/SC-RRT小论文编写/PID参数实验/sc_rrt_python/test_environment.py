"""
快速验证脚本 - 检查环境是否配置正确
Quick validation script - Check if environment is configured correctly
"""

import sys
from pathlib import Path

# 添加src到路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / 'src'))

print("=" * 70)
print("环境验证测试".center(70))
print("=" * 70)

# 测试1: 检查Python版本
print("\n[1/5] 检查Python版本...")
import sys
version = sys.version_info
if version.major >= 3 and version.minor >= 8:
    print(f"  ✓ Python {version.major}.{version.minor}.{version.micro}")
else:
    print(f"  ✗ Python版本过低: {version.major}.{version.minor}")
    sys.exit(1)

# 测试2: 检查依赖包
print("\n[2/5] 检查依赖包...")
packages = ['numpy', 'pandas', 'matplotlib', 'seaborn']
missing = []

for pkg in packages:
    try:
        __import__(pkg)
        print(f"  ✓ {pkg}")
    except ImportError:
        print(f"  ✗ {pkg} 未安装")
        missing.append(pkg)

if missing:
    print(f"\n  请安装缺失的包: pip install {' '.join(missing)}")
    sys.exit(1)

# 测试3: 导入核心模块
print("\n[3/5] 导入核心模块...")
try:
    from src.environment import EnvironmentConfig
    print("  ✓ environment")
except Exception as e:
    print(f"  ✗ environment: {e}")
    sys.exit(1)

try:
    from src.geometry import is_collision_free
    print("  ✓ geometry")
except Exception as e:
    print(f"  ✗ geometry: {e}")
    sys.exit(1)

try:
    from src.pid_controller import PIDWeightController
    print("  ✓ pid_controller")
except Exception as e:
    print(f"  ✗ pid_controller: {e}")
    sys.exit(1)

try:
    from src.sc_rrt_basic_pid import SCRRTBasicPID
    print("  ✓ sc_rrt_basic_pid")
except Exception as e:
    print(f"  ✗ sc_rrt_basic_pid: {e}")
    sys.exit(1)

# 测试4: 创建简单环境
print("\n[4/5] 测试环境生成...")
try:
    import numpy as np
    # 生成2D简单场景
    bounds = [0, 100, 0, 100]
    env_config = EnvironmentConfig.generate_2d_environment(
        bounds=bounds,
        num_obstacles=15,
        radius_range=(10.0, 20.0),
        seed=42
    )
    start = env_config['start']
    goal = env_config['goal']
    obstacles = env_config['obstacles']
    print(f"  ✓ 生成简单场景: {len(obstacles)}个障碍物")
except Exception as e:
    print(f"  ✗ 环境生成失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 测试5: 测试SC-RRT算法
print("\n[5/5] 测试SC-RRT算法...")
try:
    planner = SCRRTBasicPID(
        env=env_config,
        max_iterations=100,  # 少量迭代用于测试
        Kp=0.25,
        Ki=0.04,
        Kd=0.10,
        verbose=False
    )
    path, tree, success, metrics = planner.plan()
    
    if success:
        print(f"  ✓ 算法运行成功")
        print(f"    - 路径长度: {metrics['path_length']:.2f}")
        print(f"    - 规划时间: {metrics['planning_time']:.3f}秒")
    else:
        print(f"  ⚠ 算法未找到路径（正常，迭代次数较少）")
        print(f"    - 探索节点: {metrics['nodes_explored']}")
        
except Exception as e:
    print(f"  ✗ 算法测试失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# 全部通过
print("\n" + "=" * 70)
print("✓ 所有测试通过！环境配置正确".center(70))
print("=" * 70)
print("\n可以开始运行实验:")
print("  python run_local_pid_experiment.py --quick")
print()
