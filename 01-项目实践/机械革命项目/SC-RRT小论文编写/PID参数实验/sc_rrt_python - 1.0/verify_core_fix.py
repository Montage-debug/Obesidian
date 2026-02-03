"""验证关键修复是否已生效"""
import sys
sys.path.append('src')
sys.path.append('experiments')

print("=" * 80)
print("验证关键修复")
print("=" * 80)

# 检查修复1：椭球采样优先级
print("\n【修复1】椭球采样优先级控制")
print("-" * 80)
import inspect
from src.sc_rrt_basic_pid import SCRRTBasicPID
source = inspect.getsource(SCRRTBasicPID._sample_with_pid)

if "goal_bias_prob * 0.7" in source:
    print("❌ 仍使用 goal_bias_prob * 0.7（PID控制被削弱）")
elif "< goal_bias_prob:" in source and "* 0.7" not in source:
    print("✅ 已修复：使用 goal_bias_prob（PID完全控制椭球采样概率）")
else:
    print("⚠️  无法确定状态")

# 检查修复2：椭球放大系数
print("\n【修复2】椭球放大系数范围")
print("-" * 80)
if "ellipsoid_scale = 1.0 + (0.5 *" in source:
    print("❌ 仍使用 [1.0, 1.5] 范围（椭球过紧）")
elif "ellipsoid_scale = 1.5 + (1.0 *" in source:
    print("✅ 已修复：使用 [1.5, 2.5] 范围（考虑障碍物绕行）")
    print("   - No_PID实际椭球：5.0 × [1.5, 2.5] = 7.5-12.5倍")
    print("   - Optimal_PID实际椭球：4.0 × [1.5, 2.5] = 6.0-10.0倍")
    print("   - High_PID实际椭球：3.0 × 1.5 = 4.5倍（收紧但合理）")
else:
    print("⚠️  无法确定状态")

# 检查修复3：迭代次数
print("\n【修复3】迭代次数配置")
print("-" * 80)
from experiments.run_pid_experiment import PIDExperiment
exp = PIDExperiment(quick_test=True)
print(f"✅ 2D迭代次数: {exp.max_iterations_2d}")
print(f"✅ 3D迭代次数: {exp.max_iterations_3d}")

if exp.max_iterations_2d >= 750 and exp.max_iterations_3d >= 950:
    print("   状态：充足（避免压线成功，真实反映PID效率）")
elif exp.max_iterations_2d >= 650:
    print("   状态：适中（可能存在部分压线成功）")
else:
    print("   状态：偏紧（会掩盖PID效率优势）")

# 预期效果
print("\n【预期实验结果】")
print("=" * 80)
print("成功率梯度：")
print("  Optimal_PID:  92-95%  ★ （应该最高）")
print("  Low_PID:      88-92%")
print("  No_PID:       85-90%  （基准）")
print("  High_PID:     80-85%  （过度约束）")
print()
print("Sample-to-Success：")
print("  Optimal_PID:  400-500次  ★ （应该最快，比No_PID快20-30%）")
print("  Low_PID:      450-550次")
print("  High_PID:     500-600次")
print("  No_PID:       550-650次  （基准）")
print()
print("ESR（保持梯度）：")
print("  High_PID:     35-40%  ★ （修复后可能更高）")
print("  Optimal_PID:  18-22%")
print("  Low_PID:      12-16%")
print("  No_PID:       0.00%")
print()
print("实验规模：240次运行，预计耗时14-16分钟")
print("=" * 80)
