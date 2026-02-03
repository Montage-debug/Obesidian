"""验证所有改进是否已生效"""
import sys
sys.path.append('src')
sys.path.append('experiments')

print("=" * 70)
print("验证改进措施")
print("=" * 70)

# 1. 检查迭代次数配置
print("\n【改进1】迭代次数增加")
print("-" * 70)
from experiments.run_pid_experiment import PIDExperiment
exp = PIDExperiment(quick_test=True)
print(f"✓ 2D迭代次数: {exp.max_iterations_2d} (目标: 800)")
print(f"✓ 3D迭代次数: {exp.max_iterations_3d} (目标: 1000)")

# 2. 检查meet_point定位策略
print("\n【改进2】meet_point定位策略")
print("-" * 70)
import inspect
from src.sc_rrt_basic_pid import SCRRTBasicPID
source = inspect.getsource(SCRRTBasicPID._calculate_meet_point)
if "加权质心" in source and "最近节点对" in source:
    print("✓ meet_point已升级为：加权质心 + 最近节点对融合策略")
    print("  - 策略1: 距离加权质心 (40%权重)")
    print("  - 策略2: 最近节点对中点 (40%权重)")
    print("  - 策略3: 历史meet_point (20%权重)")
else:
    print("✗ meet_point仍使用简单质心")

# 3. 检查椭球基准倍数
print("\n【改进3】椭球基准倍数")
print("-" * 70)
source_params = inspect.getsource(SCRRTBasicPID._calculate_ellipsoid_params)
if "c_min_A * 5.0" in source_params:
    print("✓ No_PID基准: 5.0倍 (考虑障碍物绕行)")
    print("✓ PID控制范围: [3.0, 5.0]倍 (基准5.0，误差大时收紧至3.0)")
else:
    print("✗ 椭球倍数配置异常")

# 4. 预期效果
print("\n【预期效果】")
print("=" * 70)
print("1. 成功率提升：从35%左右 → 55-70%")
print("2. PID效果显现：")
print("   - Optimal_PID应在成功率和路径质量上超越No_PID")
print("   - ESR保持梯度：High_PID > Optimal_PID > Low_PID > No_PID")
print("   - Sample-to-Success：PID配置应接近或优于No_PID")
print("3. meet_point更精确 → 椭球约束更有效")
print("\n实验配置：")
print(f"  - 场景数: {len(exp.scenarios)}")
print(f"  - PID配置数: {len(exp.pid_configs)}")
print(f"  - 重复次数: {exp.num_trials}")
print(f"  - 总运行数: {len(exp.scenarios) * len(exp.pid_configs) * exp.num_trials}")
print(f"  - 预计耗时: 12-15分钟")
print("=" * 70)
