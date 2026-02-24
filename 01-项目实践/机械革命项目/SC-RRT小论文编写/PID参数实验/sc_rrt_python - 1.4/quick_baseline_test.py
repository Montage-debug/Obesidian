from experiments.run_pid_experiment import PIDExperiment

# 快速基线对比（减少重复次数以便快速验证）
exp = PIDExperiment(quick_test=True, baseline_mode=True)
exp.num_trials = 6

# 运行实验并打印每个配置的成功率
df = exp.run_experiment()
print('\n=== Success rate by PID config ===')
print(df.groupby('pid_config')['success'].mean())
