import pandas as pd
import glob
import os

csvs = sorted(glob.glob('results/pid_experiment_results_*.csv'))
if not csvs:
    print('No results CSV found in results/')
    raise SystemExit(1)

csv_path = csvs[-1]
print('Using:', csv_path)

df = pd.read_csv(csv_path)
print('\nColumns:', list(df.columns))
print('\nTotal records:', len(df))

# Overall
overall_success = df['success'].mean()
print(f'Overall success rate: {overall_success*100:.2f}%')

# Per-config aggregates
group = df.groupby('pid_config')
summary = group.agg(
    trials=('success','count'),
    success_rate=('success','mean'),
    esr_mean=('effective_sampling_ratio','mean'),
    samples_in_mean=('samples_in_ellipsoid','mean'),
    total_samples_mean=('total_samples_attempted','mean')
)

# Path length stats for successful trials
path_stats = df[df['success']==True].groupby('pid_config').agg(
    path_mean=('path_length','mean'),
    path_std=('path_length','std')
)

summary = summary.join(path_stats)
summary['esr_mean_pct'] = summary['esr_mean'] * 100

print('\nPer-config summary:')
print(summary[['trials','success_rate','esr_mean_pct','samples_in_mean','total_samples_mean','path_mean','path_std']])

# Check No_PID consistency
if 'mode' in df.columns:
    no_pid_rows = df[df['pid_config']=='No_PID']
    if len(no_pid_rows)>0:
        wrong_mode = no_pid_rows[no_pid_rows['mode']!='no_pid']
        if len(wrong_mode)>0:
            print('\nWARNING: Some No_PID rows do not have mode=="no_pid". Examples:')
            print(wrong_mode[['pid_config','mode']].drop_duplicates().head())
        else:
            print('\nNo_PID rows have mode=="no_pid" (as expected).')

    # ESR nonzero with no_pid
    no_pid_esr_nonzero = no_pid_rows[no_pid_rows['effective_sampling_ratio']>0]
    if len(no_pid_esr_nonzero)>0:
        print(f"\nWARNING: {len(no_pid_esr_nonzero)} No_PID rows have effective_sampling_ratio>0. Sample: ")
        print(no_pid_esr_nonzero[['pid_config','mode','effective_sampling_ratio']].head())
    else:
        print('\nNo_PID ESR values are all zero (good).')

# Quick flagging: if No_PID success_rate > best PID success_rate
best_pid = summary['success_rate'].idxmax()
print('\nBest config by success_rate:', best_pid)

# Save a small report
out = 'results/quick_analysis_summary.txt'
with open(out,'w',encoding='utf-8') as f:
    f.write('Using: ' + csv_path + '\n\n')
    f.write(summary.to_string())

print('\nSaved summary to', out)
