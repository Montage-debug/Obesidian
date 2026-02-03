import pandas as pd

df = pd.read_csv('results/pid_results_20260118_210901.csv')

print("="*70)
print("ESR问题诊断分析")
print("="*70)

for config in ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']:
    subset = df[df['pid_config'] == config]
    
    esr_mean = subset['effective_sampling_ratio'].mean()
    ell_mean = subset['samples_in_ellipsoid'].mean()
    tot_mean = subset['total_samples_attempted'].mean()
    esr_nonzero = (subset['effective_sampling_ratio'] > 0).sum()
    
    print(f"\n{config}:")
    print(f"  平均ESR: {esr_mean*100:.2f}%")
    print(f"  平均椭球内采样: {ell_mean:.1f} / {tot_mean:.1f}")
    print(f"  实际比例: {ell_mean/tot_mean*100:.2f}%")
    print(f"  有ESR数据的样本: {esr_nonzero}/{len(subset)}")
    
    if config != 'No_PID':
        # 查看成功和失败样本的ESR差异
        success_esr = subset[subset['success']==True]['effective_sampling_ratio'].mean()
        fail_esr = subset[subset['success']==False]['effective_sampling_ratio'].mean()
        print(f"  成功样本ESR: {success_esr*100:.2f}%")
        print(f"  失败样本ESR: {fail_esr*100:.2f}%")

print("\n" + "="*70)
print("问题分析：ESR为什么这么低？")
print("="*70)

# 检查一个Optimal_PID的成功样本
optimal_success = df[(df['pid_config']=='Optimal_PID') & (df['success']==True)].iloc[0]
print(f"\n一个成功样本的详细数据：")
print(f"  samples_in_ellipsoid: {optimal_success['samples_in_ellipsoid']}")
print(f"  total_samples_attempted: {optimal_success['total_samples_attempted']}")
print(f"  effective_sampling_ratio: {optimal_success['effective_sampling_ratio']:.4f}")
print(f"  iterations: {optimal_success['iterations']}")
print(f"  tree_nodes: {optimal_success['tree_nodes']}")

print("\n可能的原因：")
print("1. c_best在大部分时间都是inf，所以大部分采样点不算在椭球内")
print("2. 只有找到第一条路径后，c_best才变成有限值")
print("3. 之后的采样才可能被计入ESR")
