"""快速ESR修复验证 - 单次运行测试"""
import sys
import numpy as np
sys.path.insert(0, 'e:\\Matlab练习\\RRT对比实验 -3.2 转python做pid对比实验\\sc_rrt_python')
from src.sc_rrt_basic_pid import SCRRTBasicPID

# 二维高密度场景
dim = 2
start = np.array([1.0, 1.0])
goal = np.array([19.0, 19.0])
lower = np.array([0.0, 0.0])
upper = np.array([20.0, 20.0])

# 生成障碍物
np.random.seed(42)
num_obstacles = 150
obstacles = []
for _ in range(num_obstacles):
    center = np.random.uniform([2, 2], [18, 18])
    radius = np.random.uniform(0.3, 0.7)
    obstacles.append({'center': center, 'radius': radius})

print("=" * 70)
print("快速ESR修复验证 - 测试PID控制器是否正常引导采样")
print("=" * 70)

configs = [
    ("No_PID", 0.0, 0.0, 0.0),
    ("Optimal_PID", 0.25, 0.04, 0.06),
]

for config_name, Kp, Ki, Kd in configs:
    print(f"\n【{config_name}】")
    
    env = {
        'start': start, 'goal': goal, 'obstacles': obstacles,
        'lower_bound': lower, 'upper_bound': upper,
        'dimension': dim
    }
    
    planner = SCRRTBasicPID(
        env=env,
        Kp=Kp, Ki=Ki, Kd=Kd,
        max_iterations=600
    )
    
    path, tree, success, metrics = planner.plan()

    if success:
        esr = metrics.get('effective_sampling_ratio', 0.0) * 100
        samples_in = metrics.get('samples_in_ellipsoid', 0)
        total_samples = metrics.get('total_samples_attempted', 0)

        print(f"  ✓ 规划成功")
        print(f"  ESR: {esr:.2f}% ({samples_in}/{total_samples} 椭球内采样)")
        print(f"  路径长度: {metrics['path_length']:.2f}")
        print(f"  迭代次数: {metrics['iterations']}")
        
        # 检查是否符合预期
        if config_name == "No_PID":
            if esr < 5.0:
                print(f"  ✅ No_PID的ESR应接近0% - 通过")
            else:
                print(f"  ❌ No_PID的ESR过高: {esr:.2f}%")
        else:
            if esr >= 40.0:
                print(f"  ✅ Optimal_PID的ESR应>40% - 通过")
            else:
                print(f"  ⚠️ Optimal_PID的ESR仍偏低: {esr:.2f}% (目标55-70%)")
    else:
        print(f"  ✗ 规划失败")

print("\n" + "=" * 70)
print("验证完成")
print("=" * 70)
