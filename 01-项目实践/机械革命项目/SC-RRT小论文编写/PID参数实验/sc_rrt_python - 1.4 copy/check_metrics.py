import pandas as pd

data = pd.read_csv('results/pid_results_20260118_232931.csv')
success_data = data[data['success'] == True]

print('=== Sample-to-Success (成功样本，用iterations代替) ===')
for config in ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']:
    subset = success_data[success_data['pid_config'] == config]
    if len(subset) > 0:
        print(f'{config}: mean={subset["iterations"].mean():.1f}, median={subset["iterations"].median():.0f}')

print('\n=== ESR (所有样本) ===')
for config in ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']:
    subset = data[data['pid_config'] == config]
    print(f'{config}: mean={subset["effective_sampling_ratio"].mean()*100:.2f}%')

print('\n=== 成功率 ===')
for config in ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']:
    subset = data[data['pid_config'] == config]
    success_count = (subset['success'] == True).sum()
    total_count = len(subset)
    print(f'{config}: {success_count}/{total_count} = {success_count/total_count*100:.1f}%')
