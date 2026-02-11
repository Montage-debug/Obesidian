import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，避免GUI问题
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd  # 添加pandas用于读取CSV
import seaborn as sns
from matplotlib import rcParams
import matplotlib.font_manager as fm
import os
import shutil
from pathlib import Path

# 清除matplotlib字体缓存
def clear_font_cache():
    """清除matplotlib的字体缓存，强制重新加载"""
    try:
        cache_dir = matplotlib.get_cachedir()
        cache_files = ['fontlist-v330.json', 'fontlist-v310.json', 'fontList.cache']
        for cache_file in cache_files:
            full_path = os.path.join(cache_dir, cache_file)
            if os.path.exists(full_path):
                try:
                    os.remove(full_path)
                    print(f"✅ 已清除缓存文件: {cache_file}")
                except Exception as e:
                    print(f"⚠️  无法删除缓存: {e}")
    except Exception as e:
        print(f"⚠️  清除缓存失败: {e}")

# 查找系统中可用的中文字体
def get_chinese_font():
    """自动检测系统中可用的中文字体"""
    chinese_fonts = ['Microsoft YaHei', 'SimHei', 'SimSun', 'KaiTi', 'FangSong']
    
    # 刷新字体列表
    fm._load_fontmanager(try_read_cache=False)
    available_fonts = sorted([f.name for f in fm.fontManager.ttflist])
    
    # 打印前20个字体用于调试
    print(f"📝 系统中前20个可用字体: {available_fonts[:20]}")
    
    for font in chinese_fonts:
        if font in available_fonts:
            print(f"✅ 使用中文字体: {font}")
            return font
    
    # 如果没找到，尝试查找包含'Chinese'或'CJK'的字体
    for font_name in available_fonts:
        if 'Chinese' in font_name or 'CJK' in font_name or 'YaHei' in font_name or 'SimHei' in font_name:
            print(f"✅ 找到替代中文字体: {font_name}")
            return font_name
    
    # 最后尝试使用系统默认字体
    print("⚠️  未找到中文字体，将使用系统默认字体（中文可能无法显示）")
    return 'sans-serif'

# 清除缓存
clear_font_cache()

# 配置中文字体
chinese_font = get_chinese_font()

# 使用简单的样式设置（必须在字体设置之前）
plt.style.use('default')
sns.set_palette("husl")
colors = ['#2E86AB', '#A23B72', '#F18F01', '#C73E1D', '#6A994E']

# 全局设置字体（必须在style.use之后，否则会被覆盖）
plt.rcParams['font.sans-serif'] = [chinese_font]
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'sans-serif'

# 再次确保所有相关配置都使用中文字体
rcParams.update({
    'font.sans-serif': [chinese_font],
    'axes.unicode_minus': False,
    'font.size': 10,
    'font.family': 'sans-serif',
    'text.usetex': False,
    # 强制所有文本元素使用配置的字体
    'mathtext.fontset': 'custom',
    'mathtext.rm': chinese_font,
    'mathtext.it': chinese_font,
    'mathtext.bf': chinese_font,
})

print(f"🎨 最终字体配置: {plt.rcParams['font.sans-serif']}")

# ====================== 数据准备 ======================
# 1. Stage 1 Top10 数据
stage1_rank = np.arange(1, 11)
stage1_kp = [0.16, 0.16, 0.16, 0.05, 0.28, 0.05, 0.45, 0.05, 0.28, 0.28]
stage1_ki = [0.026, 0.018, 0.026, 0.070, 0.040, 0.026, 0.018, 0.034, 0.000, 0.026]
stage1_kd = [0.080, 0.095, 0.030, 0.030, 0.030, 0.080, 0.080, 0.080, 0.080, 0.095]
stage1_path = [1041.42, 1074.37, 1089.45, 1094.38, 1095.37, 1095.61, 1095.77, 1097.59, 1102.92, 1106.00]
stage1_std = [6.41, 35.42, 54.11, 16.90, 49.82, 78.21, 12.92, 17.56, 79.80, 28.25]
stage1_time = [8.72, 8.62, 8.64, 8.34, 6.19, 8.61, 8.45, 8.47, 8.52, 8.54]

# 2. Stage 2 Top10 数据
stage2_rank = np.arange(1, 11)
stage2_kp = [0.26, 0.10, 0.26, 0.21, 0.16, 0.24, 0.23, 0.23, 0.25, 0.22]
stage2_ki = [0.024, 0.026, 0.038, 0.018, 0.040, 0.020, 0.034, 0.035, 0.026, 0.035]
stage2_kd = [0.060, 0.095, 0.075, 0.090, 0.065, 0.065, 0.100, 0.085, 0.090, 0.055]
stage2_path = [1075.77, 1077.99, 1078.06, 1080.99, 1081.93, 1083.36, 1083.76, 1087.80, 1088.08, 1088.15]
stage2_std = [54.76, 19.33, 31.33, 63.32, 35.17, 29.91, 25.97, 47.84, 61.14, 59.51]
stage2_time = [11.82, 12.80, 12.25, 11.33, 13.07, 12.39, 12.60, 9.93, 10.18, 9.80]

# 3. Stage 3 真实数据（从CSV读取）
stage3_file = Path("sc_rrt_python - 1.4/results").glob("stage3_validation_*.csv")
stage3_files = sorted(stage3_file)
if stage3_files:
    stage3_df = pd.read_csv(stage3_files[-1])
    stage3_successful = stage3_df[stage3_df['success'] == True]
    stage3_path_data = stage3_successful['path_length'].values
    stage3_mean = stage3_path_data.mean()
    stage3_std = stage3_path_data.std()
    print(f"✅ 加载Stage 3真实数据: {len(stage3_path_data)}次运行, 均值{stage3_mean:.2f}m")
else:
    # 如果没有Stage 3数据，使用Stage 2最优的5次数据
    print("⚠️  Stage 3数据未找到，使用Stage 2数据")
    stage3_path_data = np.array([stage2_path[0]] * 5)
    stage3_mean = stage2_path[0]
    stage3_std = stage2_std[0]

# 4. 参数敏感性分析数据
param_sensitivity = {
    'Kp': {'optimal': 0.26, 'range': [0.16, 0.28], 'drop10': [0.15, 0.30], 'sensitivity': '中等'},
    'Ki': {'optimal': 0.024, 'range': [0.018, 0.040], 'drop10': [0.015, 0.050], 'sensitivity': '较高'},
    'Kd': {'optimal': 0.060, 'range': [0.055, 0.095], 'drop10': [0.050, 0.110], 'sensitivity': '较低'}
}

# ====================== 图表绘制 ======================
fig = plt.figure(figsize=(20, 16))

# 子图1: Stage1 vs Stage2 Top10 平均路径长度对比
ax1 = plt.subplot(2, 2, 1)
x = np.arange(1, 11)
width = 0.35

bars1 = ax1.bar(x - width/2, stage1_path, width, label='Stage 1 粗搜索', color=colors[0], alpha=0.8)
bars2 = ax1.bar(x + width/2, stage2_path, width, label='Stage 2 精细调优', color=colors[1], alpha=0.8)
ax1.plot(x, [np.mean(stage1_path)]*10, '--', color=colors[0], label='Stage 1 均值')
ax1.plot(x, [np.mean(stage2_path)]*10, '--', color=colors[1], label='Stage 2 均值')

ax1.set_xlabel('排名', fontsize=12)
ax1.set_ylabel('平均路径长度 (像素)', fontsize=12)
ax1.set_title('Stage 1 vs Stage 2 Top10 平均路径长度对比', fontsize=14, fontweight='bold')
ax1.set_xticks(x)
ax1.legend()
ax1.grid(True, alpha=0.3)

# 添加数值标签
for bar in bars1:
    height = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width()/2., height, f'{height:.1f}', ha='center', va='bottom', fontsize=8)
for bar in bars2:
    height = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width()/2., height, f'{height:.1f}', ha='center', va='bottom', fontsize=8)

# 子图2: 最优PID参数Stage3性能分布（箱线图）
ax2 = plt.subplot(2, 2, 2)
box_plot = ax2.boxplot(stage3_path_data, patch_artist=True, 
                       boxprops=dict(facecolor=colors[2], alpha=0.8),
                       medianprops=dict(color='black', linewidth=2),
                       whiskerprops=dict(color=colors[2]),
                       capprops=dict(color=colors[2]))

# 添加均值点和置信区间
mean_val = np.mean(stage3_path_data)
ci_95 = 1.96 * np.std(stage3_path_data) / np.sqrt(len(stage3_path_data))
ax2.scatter(1, mean_val, color='red', s=100, marker='*', label=f'均值: {mean_val:.2f}')
ax2.errorbar(1, mean_val, yerr=ci_95, fmt='none', ecolor='red', capsize=10, label=f'95%置信区间: ±{ci_95:.2f}')

ax2.set_ylabel('路径长度 (像素)', fontsize=12)
ax2.set_title('最优PID参数(Stage3)路径长度分布 (n=20)', fontsize=14, fontweight='bold')
ax2.set_xticklabels(['最优配置(Kp=0.26, Ki=0.024, Kd=0.060)'])
ax2.legend()
ax2.grid(True, alpha=0.3)

# 子图3: PID参数敏感性分析
ax3 = plt.subplot(2, 2, 3)
params = ['Kp', 'Ki', 'Kd']
optimal_vals = [param_sensitivity[p]['optimal'] for p in params]
lower_range = [param_sensitivity[p]['range'][0] for p in params]
upper_range = [param_sensitivity[p]['range'][1] for p in params]
lower_drop10 = [param_sensitivity[p]['drop10'][0] for p in params]
upper_drop10 = [param_sensitivity[p]['drop10'][1] for p in params]

x_pos = np.arange(len(params))
width = 0.15

ax3.bar(x_pos - 2*width, lower_drop10, width, label='性能下降10%下限', color=colors[3], alpha=0.7)
ax3.bar(x_pos - width, lower_range, width, label='有效范围下限', color=colors[0], alpha=0.7)
ax3.bar(x_pos, optimal_vals, width, label='最优值', color=colors[2], alpha=1.0)
ax3.bar(x_pos + width, upper_range, width, label='有效范围上限', color=colors[1], alpha=0.7)
ax3.bar(x_pos + 2*width, upper_drop10, width, label='性能下降10%上限', color=colors[3], alpha=0.7)

ax3.set_xlabel('PID参数', fontsize=12)
ax3.set_ylabel('参数值', fontsize=12)
ax3.set_title('PID参数敏感性分析', fontsize=14, fontweight='bold')
ax3.set_xticks(x_pos)
ax3.set_xticklabels(params)
ax3.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
ax3.grid(True, alpha=0.3)

# 子图4: 不同阶段最优参数核心指标对比
ax4 = plt.subplot(2, 2, 4)
stages = ['Stage1最优', 'Stage2最优', 'Stage3验证']
avg_path = [1041.42, 1075.77, 1180.20]
std_path = [6.41, 54.76, 66.80]
avg_time = [8.72, 11.82, 9.577]

x = np.arange(len(stages))
width = 0.25

bars1 = ax4.bar(x - width, avg_path, width, label='平均路径长度', color=colors[0], alpha=0.8)
bars2 = ax4.bar(x, std_path, width, label='路径长度标准差', color=colors[1], alpha=0.8)
bars3 = ax4.bar(x + width, avg_time, width, label='平均运行时间(s)', color=colors[2], alpha=0.8)

ax4.set_xlabel('优化阶段', fontsize=12)
ax4.set_ylabel('数值', fontsize=12)
ax4.set_title('不同阶段最优参数核心性能指标', fontsize=14, fontweight='bold')
ax4.set_xticks(x)
ax4.set_xticklabels(stages)
ax4.legend()
ax4.grid(True, alpha=0.3)

# 添加数值标签
for bar in bars1:
    height = bar.get_height()
    ax4.text(bar.get_x() + bar.get_width()/2., height, f'{height:.1f}', ha='center', va='bottom', fontsize=8)
for bar in bars2:
    height = bar.get_height()
    ax4.text(bar.get_x() + bar.get_width()/2., height, f'{height:.1f}', ha='center', va='bottom', fontsize=8)
for bar in bars3:
    height = bar.get_height()
    ax4.text(bar.get_x() + bar.get_width()/2., height, f'{height:.2f}', ha='center', va='bottom', fontsize=8)

# 整体布局调整
plt.tight_layout()
#rint("✅ 第一张图已保存: PID参数优化实验结果可视化.png"
plt.savefig('PID参数优化实验结果可视化.png', dpi=300, bbox_inches='tight')
plt.show()

# ====================== 补充分析：PID参数与路径长度的相关性 ======================
fig2, axes = plt.subplots(1, 3, figsize=(18, 6))

# Kp vs 平均路径长度
axes[0].scatter(stage1_kp, stage1_path, color=colors[0], label='Stage1', alpha=0.8, s=60)
axes[0].scatter(stage2_kp, stage2_path, color=colors[1], label='Stage2', alpha=0.8, s=60)
axes[0].axvline(x=0.26, color='red', linestyle='--', label='最优Kp=0.26')
axes[0].set_xlabel('比例系数 Kp', fontsize=12)
axes[0].set_ylabel('平均路径长度', fontsize=12)
axes[0].set_title('Kp 与平均路径长度的关系', fontsize=14, fontweight='bold')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# Ki vs 平均路径长度
axes[1].scatter(stage1_ki, stage1_path, color=colors[0], label='Stage1', alpha=0.8, s=60)
axes[1].scatter(stage2_ki, stage2_path, color=colors[1], label='Stage2', alpha=0.8, s=60)
axes[1].axvline(x=0.024, color='red', linestyle='--', label='最优Ki=0.024')
axes[1].set_xlabel('积分系数 Ki', fontsize=12)
axes[1].set_ylabel('平均路径长度', fontsize=12)
axes[1].set_title('Ki 与平均路径长度的关系', fontsize=14, fontweight='bold')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# Kd vs 平均路径长度
axes[2].scatter(stage1_kd, stage1_path, color=colors[0], label='Stage1', alpha=0.8, s=60)
axes[2].scatter(stage2_kd, stage2_path, color=colors[1], label='Stage2', alpha=0.8, s=60)
axes[2].axvline(x=0.060, color='red', linestyle='--', label='最优Kd=0.060')
axes[2].set_xlabel('微分系数 Kd', fontsize=12)
axes[2].set_ylabel('平均路径长度', fontsize=12)
axes[2].set_title('Kd 与平均路径长度的关系', fontsize=14, fontweight='bold')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
# 保存到当前目录
plt.savefig('PID参数与路径长度相关性.png', dpi=300, bbox_inches='tight')
print("✅ 第二张图已保存: PID参数与路径长度相关性.png")
print("\n🎉 所有图表生成完成！请查看当前目录下的两张PNG图片。")