# -*- coding: utf-8 -*-
"""
读取真实CSV数据生成论文所需图表
Version: 2.0 - Data-driven from CSV files
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互式后端
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# ====================== 字体配置 ======================
# 1. 清理字体缓存
import shutil
cache_dir = matplotlib.get_cachedir()
if Path(cache_dir).exists():
    try:
        shutil.rmtree(cache_dir)
        print(f"✓ 已清理matplotlib字体缓存: {cache_dir}")
    except Exception as e:
        print(f"⚠️  无法清理字体缓存: {e}")

# 2. 先应用样式（避免后续覆盖字体设置）
plt.style.use('default')

# 3. 配置中文字体
from matplotlib.font_manager import FontProperties, findSystemFonts
import matplotlib.font_manager as fm

# 查找系统中的中文字体
def find_chinese_font():
    """自动查找可用的中文字体"""
    chinese_fonts = ['Microsoft YaHei', 'SimHei', 'SimSun', 'KaiTi', 'FangSong']
    
    system_fonts = findSystemFonts(fontpaths=None, fontext='ttf')
    
    for font_name in chinese_fonts:
        for font_path in system_fonts:
            if font_name.lower() in font_path.lower():
                print(f"✓ 找到中文字体: {font_name} -> {font_path}")
                return font_name
    
    print("⚠️  未找到中文字体，将使用默认字体")
    return None

chinese_font = find_chinese_font()

if chinese_font:
    plt.rcParams['font.sans-serif'] = [chinese_font, 'Arial', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    print(f"✓ 字体配置完成: {chinese_font}")
else:
    print("⚠️  使用默认字体配置")

# ====================== 加载真实数据 ======================
print("\n" + "="*80)
print("从CSV文件加载实验数据")
print("="*80)

results_dir = Path("results")

# 1. 加载Stage 1数据
stage1_file = results_dir / "stage1_coarse_20260209_230704.csv"
if not stage1_file.exists():
    raise FileNotFoundError(f"Stage 1数据文件未找到: {stage1_file}")

stage1_df = pd.read_csv(stage1_file)
print(f"✓ Stage 1: {len(stage1_df)} 配置")

# 提取Top 10
stage1_top10 = stage1_df.head(10)
stage1_kp = stage1_top10['Kp'].values
stage1_ki = stage1_top10['Ki'].values
stage1_kd = stage1_top10['Kd'].values
stage1_path = stage1_top10['avg_length'].values
stage1_std = stage1_top10['std_length'].values
stage1_time = stage1_top10['avg_time'].values

# 2. 加载Stage 2数据
stage2_file = results_dir / "stage2_fine_20260209_230704.csv"
if not stage2_file.exists():
    raise FileNotFoundError(f"Stage 2数据文件未找到: {stage2_file}")

stage2_df = pd.read_csv(stage2_file)
print(f"✓ Stage 2: {len(stage2_df)} 配置")

# 提取Top 10
stage2_top10 = stage2_df.head(10)
stage2_kp = stage2_top10['Kp'].values
stage2_ki = stage2_top10['Ki'].values
stage2_kd = stage2_top10['Kd'].values
stage2_path = stage2_top10['avg_length'].values
stage2_std = stage2_top10['std_length'].values
stage2_time = stage2_top10['avg_time'].values

# 3. 加载Stage 3数据
stage3_files = sorted(results_dir.glob("stage3_validation_*.csv"))
if not stage3_files:
    print("⚠️  Stage 3数据未找到，使用Stage 2最优配置数据")
    stage3_path_data = np.array([stage2_path[0]] * 20)  # 使用Stage 2最优值
    stage3_available = False
else:
    stage3_file = stage3_files[-1]  # 使用最新的
    stage3_df = pd.read_csv(stage3_file)
    print(f"✓ Stage 3: {stage3_file.name}, {len(stage3_df)} 运行")
    
    # 只使用成功的运行
    stage3_successful = stage3_df[stage3_df['success'] == True]
    if len(stage3_successful) == 0:
        print("⚠️  Stage 3无成功运行，使用Stage 2最优配置数据")
        stage3_path_data = np.array([stage2_path[0]] * 20)
        stage3_available = False
    else:
        stage3_path_data = stage3_successful['path_length'].values
        stage3_available = True
        print(f"  成功率: {len(stage3_successful)}/{len(stage3_df)} ({len(stage3_successful)/len(stage3_df)*100:.1f}%)")
        print(f"  路径长度: {stage3_path_data.mean():.2f} ± {stage3_path_data.std():.2f}")

# 4. 参数敏感性分析（从实际数据计算）
print("\n计算参数敏感性...")

# 从Stage 2数据中分析每个参数的影响
def calculate_sensitivity(df, param_name, optimal_value):
    """计算参数敏感性"""
    # 找到与最优值接近的配置范围
    param_range = df[param_name].values
    path_lengths = df['avg_length'].values
    
    # 计算不同参数值下的性能
    sorted_indices = np.argsort(path_lengths)
    top_20pct_indices = sorted_indices[:int(len(sorted_indices)*0.2)]
    
    top_values = df.iloc[top_20pct_indices][param_name].values
    param_range_good = [top_values.min(), top_values.max()]
    
    # 找出性能下降10%的参数范围
    threshold_length = path_lengths[sorted_indices[0]] * 1.10
    acceptable_indices = path_lengths <= threshold_length
    acceptable_values = df[acceptable_indices][param_name].values
    
    if len(acceptable_values) > 0:
        drop10_range = [acceptable_values.min(), acceptable_values.max()]
    else:
        drop10_range = param_range_good
    
    # 计算敏感性评分
    variation_coef = np.std(top_values) / np.mean(top_values)
    if variation_coef < 0.15:
        sensitivity = '较低'
    elif variation_coef < 0.30:
        sensitivity = '中等'
    else:
        sensitivity = '较高'
    
    return {
        'optimal': optimal_value,
        'range': param_range_good,
        'drop10': drop10_range,
        'sensitivity': sensitivity
    }

optimal_kp = stage2_kp[0]
optimal_ki = stage2_ki[0]
optimal_kd = stage2_kd[0]

param_sensitivity = {
    'Kp': calculate_sensitivity(stage2_df, 'Kp', optimal_kp),
    'Ki': calculate_sensitivity(stage2_df, 'Ki', optimal_ki),
    'Kd': calculate_sensitivity(stage2_df, 'Kd', optimal_kd)
}

print(f"  Kp: 最优={param_sensitivity['Kp']['optimal']:.2f}, "
      f"范围={param_sensitivity['Kp']['range']}, "
      f"敏感性={param_sensitivity['Kp']['sensitivity']}")
print(f"  Ki: 最优={param_sensitivity['Ki']['optimal']:.3f}, "
      f"范围={param_sensitivity['Ki']['range']}, "
      f"敏感性={param_sensitivity['Ki']['sensitivity']}")
print(f"  Kd: 最优={param_sensitivity['Kd']['optimal']:.2f}, "
      f"范围={param_sensitivity['Kd']['range']}, "
      f"敏感性={param_sensitivity['Kd']['sensitivity']}")

print("\n数据加载完成！\n")

# ====================== 图表绘制 ======================
fig = plt.figure(figsize=(20, 16))

# 调整整体布局
fig.suptitle('SC-RRT算法PID参数优化实验结果', fontsize=24, fontweight='bold', y=0.98)

# ------------------------ 子图1: 两阶段Top 10路径长度对比 ------------------------
ax1 = plt.subplot(2, 2, 1)

x_positions_s1 = np.arange(1, 11)
x_positions_s2 = x_positions_s1 + 0.35

bars1 = ax1.bar(x_positions_s1, stage1_path, width=0.35, 
                label='Stage 1 (粗搜索)', color='#3498db', alpha=0.8, edgecolor='black')
bars2 = ax1.bar(x_positions_s2, stage2_path, width=0.35,
                label='Stage 2 (精调)', color='#e74c3c', alpha=0.8, edgecolor='black')

# 添加误差线
ax1.errorbar(x_positions_s1, stage1_path, yerr=stage1_std, fmt='none', 
             ecolor='black', capsize=4, alpha=0.6)
ax1.errorbar(x_positions_s2, stage2_path, yerr=stage2_std, fmt='none',
             ecolor='black', capsize=4, alpha=0.6)

ax1.set_xlabel('配置排名', fontsize=14, fontweight='bold')
ax1.set_ylabel('平均路径长度 (m)', fontsize=14, fontweight='bold')
ax1.set_title('(a) 两阶段优化Top 10配置性能对比', fontsize=16, fontweight='bold', pad=15)
ax1.set_xticks(x_positions_s1 + 0.175)
ax1.set_xticklabels(range(1, 11))
ax1.legend(fontsize=12, loc='upper left', framealpha=0.9)
ax1.grid(axis='y', linestyle='--', alpha=0.3)

# 标注最优值
best_s1_idx = 0
best_s2_idx = 0
ax1.annotate(f'{stage1_path[best_s1_idx]:.2f}m', 
            xy=(x_positions_s1[best_s1_idx], stage1_path[best_s1_idx]),
            xytext=(x_positions_s1[best_s1_idx], stage1_path[best_s1_idx] + 30),
            ha='center', fontsize=11, fontweight='bold', color='#2c3e50',
            arrowprops=dict(arrowstyle='->', color='#2c3e50', lw=1.5))
ax1.annotate(f'{stage2_path[best_s2_idx]:.2f}m', 
            xy=(x_positions_s2[best_s2_idx], stage2_path[best_s2_idx]),
            xytext=(x_positions_s2[best_s2_idx], stage2_path[best_s2_idx] + 30),
            ha='center', fontsize=11, fontweight='bold', color='#c0392b',
            arrowprops=dict(arrowstyle='->', color='#c0392b', lw=1.5))

# ------------------------ 子图2: PID参数变化趋势 ------------------------
ax2 = plt.subplot(2, 2, 2)

x = np.arange(1, 11)

# 归一化PID参数到相同量级以便对比
kp_norm = stage2_kp / stage2_kp.max()
ki_norm = stage2_ki / stage2_ki.max()
kd_norm = stage2_kd / stage2_kd.max()

ax2.plot(x, kp_norm, marker='o', markersize=8, linewidth=2.5, 
         label=f'Kp (最优={stage2_kp[0]:.2f})', color='#e74c3c', alpha=0.85)
ax2.plot(x, ki_norm, marker='s', markersize=8, linewidth=2.5,
         label=f'Ki (最优={stage2_ki[0]:.3f})', color='#3498db', alpha=0.85)
ax2.plot(x, kd_norm, marker='^', markersize=8, linewidth=2.5,
         label=f'Kd (最优={stage2_kd[0]:.2f})', color='#2ecc71', alpha=0.85)

ax2.set_xlabel('配置排名', fontsize=14, fontweight='bold')
ax2.set_ylabel('归一化参数值', fontsize=14, fontweight='bold')
ax2.set_title('(b) Stage 2 Top 10 PID参数变化趋势', fontsize=16, fontweight='bold', pad=15)
ax2.set_xticks(x)
ax2.legend(fontsize=12, loc='best', framealpha=0.9)
ax2.grid(True, linestyle='--', alpha=0.3)

# 标注最优点
ax2.scatter([1], [kp_norm[0]], s=150, color='#e74c3c', marker='*', edgecolors='black', linewidths=1.5, zorder=5)
ax2.scatter([1], [ki_norm[0]], s=150, color='#3498db', marker='*', edgecolors='black', linewidths=1.5, zorder=5)
ax2.scatter([1], [kd_norm[0]], s=150, color='#2ecc71', marker='*', edgecolors='black', linewidths=1.5, zorder=5)

# ------------------------ 子图3: Stage 3最优配置性能分布 ------------------------
ax3 = plt.subplot(2, 2, 3)

# 绘制直方图
n, bins, patches = ax3.hist(stage3_path_data, bins=15, color='#9b59b6', alpha=0.7, 
                             edgecolor='black', linewidth=1.2)

# 添加核密度估计曲线
from scipy.stats import gaussian_kde
if len(stage3_path_data) > 1:
    density = gaussian_kde(stage3_path_data)
    xs = np.linspace(stage3_path_data.min(), stage3_path_data.max(), 200)
    ys = density(xs)
    # 将密度缩放到直方图的尺度
    ys_scaled = ys * len(stage3_path_data) * (bins[1] - bins[0])
    ax3.plot(xs, ys_scaled, 'r-', linewidth=2.5, label='核密度估计')

# 标注统计信息
mean_path = stage3_path_data.mean()
std_path = stage3_path_data.std()
ax3.axvline(mean_path, color='red', linestyle='--', linewidth=2, label=f'均值={mean_path:.2f}m')
ax3.axvline(mean_path - std_path, color='orange', linestyle=':', linewidth=1.5, alpha=0.7)
ax3.axvline(mean_path + std_path, color='orange', linestyle=':', linewidth=1.5, alpha=0.7, 
            label=f'±1σ (σ={std_path:.2f}m)')

ax3.set_xlabel('路径长度 (m)', fontsize=14, fontweight='bold')
ax3.set_ylabel('频数', fontsize=14, fontweight='bold')

if stage3_available:
    title_suffix = f'(Kp={optimal_kp:.2f}, Ki={optimal_ki:.3f}, Kd={optimal_kd:.2f})'
else:
    title_suffix = '(待运行)'

ax3.set_title(f'(c) Stage 3 最优配置性能分布 {title_suffix}', 
              fontsize=16, fontweight='bold', pad=15)
ax3.legend(fontsize=12, loc='upper right', framealpha=0.9)
ax3.grid(axis='y', linestyle='--', alpha=0.3)

# 添加文本框显示详细统计
textstr = f'样本数: {len(stage3_path_data)}\n'
textstr += f'均值: {mean_path:.2f} m\n'
textstr += f'标准差: {std_path:.2f} m\n'
textstr += f'最小值: {stage3_path_data.min():.2f} m\n'
textstr += f'最大值: {stage3_path_data.max():.2f} m'

props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
ax3.text(0.05, 0.95, textstr, transform=ax3.transAxes, fontsize=11,
        verticalalignment='top', bbox=props)

# ------------------------ 子图4: 参数敏感性分析 ------------------------
ax4 = plt.subplot(2, 2, 4)

params = ['Kp', 'Ki', 'Kd']
colors = ['#e74c3c', '#3498db', '#2ecc71']

for i, param in enumerate(params):
    data = param_sensitivity[param]
    optimal = data['optimal']
    range_good = data['range']
    range_10pct = data['drop10']
    
    y_pos = len(params) - i - 1
    
    # 绘制性能下降10%的容忍范围（浅色）
    ax4.barh(y_pos, range_10pct[1] - range_10pct[0], left=range_10pct[0], 
             height=0.6, color=colors[i], alpha=0.3, edgecolor='black', linewidth=1)
    
    # 绘制高性能范围（深色）
    ax4.barh(y_pos, range_good[1] - range_good[0], left=range_good[0],
             height=0.4, color=colors[i], alpha=0.8, edgecolor='black', linewidth=1.5)
    
    # 标注最优值
    ax4.plot([optimal], [y_pos], marker='D', markersize=12, color='red', 
             markeredgecolor='black', markeredgewidth=1.5, zorder=5)
    
    # 标注数值
    ax4.text(optimal, y_pos + 0.35, f'{optimal:.3f}' if param == 'Ki' else f'{optimal:.2f}',
             ha='center', va='bottom', fontsize=11, fontweight='bold', color='darkred')
    
    # 标注敏感性
    sensitivity_text = f"敏感性: {data['sensitivity']}"
    ax4.text(range_10pct[1] + 0.02, y_pos, sensitivity_text, 
             va='center', fontsize=10, color='black', fontstyle='italic')

ax4.set_yticks(range(len(params)))
ax4.set_yticklabels(params, fontsize=14, fontweight='bold')
ax4.set_xlabel('参数值', fontsize=14, fontweight='bold')
ax4.set_title('(d) PID参数敏感性分析', fontsize=16, fontweight='bold', pad=15)
ax4.grid(axis='x', linestyle='--', alpha=0.3)

# 添加图例
from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor='gray', alpha=0.3, edgecolor='black', label='性能下降10%容忍范围'),
    Patch(facecolor='gray', alpha=0.8, edgecolor='black', label='高性能参数范围'),
    plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='red', 
               markeredgecolor='black', markersize=10, label='最优值')
]
ax4.legend(handles=legend_elements, fontsize=11, loc='lower right', framealpha=0.9)

# ====================== 调整布局并保存 ======================
plt.tight_layout(rect=[0, 0, 1, 0.97])

output_file = Path("results/PID参数优化实验结果可视化_真实数据.png")
plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
print(f"✓ 图表1已保存: {output_file}")
plt.close()

# ====================== 图表2: 参数相关性散点图 ======================
fig2 = plt.figure(figsize=(18, 12))
fig2.suptitle('PID参数与路径长度相关性分析', fontsize=22, fontweight='bold', y=0.98)

# 使用Stage 2完整数据进行相关性分析
scatter_size = 50
scatter_alpha = 0.6

# Kp vs 路径长度
ax5 = plt.subplot(2, 3, 1)
scatter1 = ax5.scatter(stage2_df['Kp'], stage2_df['avg_length'], 
                      c=stage2_df['avg_length'], cmap='coolwarm', 
                      s=scatter_size, alpha=scatter_alpha, edgecolors='black', linewidth=0.5)
ax5.scatter(optimal_kp, stage2_path[0], s=300, c='red', marker='*', 
           edgecolors='black', linewidths=2, zorder=5, label='最优配置')
ax5.set_xlabel('Kp', fontsize=13, fontweight='bold')
ax5.set_ylabel('路径长度 (m)', fontsize=13, fontweight='bold')
ax5.set_title('Kp与路径长度关系', fontsize=14, fontweight='bold')
ax5.grid(True, linestyle='--', alpha=0.3)
ax5.legend(fontsize=10)
plt.colorbar(scatter1, ax=ax5, label='路径长度(m)')

# Ki vs 路径长度
ax6 = plt.subplot(2, 3, 2)
scatter2 = ax6.scatter(stage2_df['Ki'], stage2_df['avg_length'],
                      c=stage2_df['avg_length'], cmap='coolwarm',
                      s=scatter_size, alpha=scatter_alpha, edgecolors='black', linewidth=0.5)
ax6.scatter(optimal_ki, stage2_path[0], s=300, c='red', marker='*',
           edgecolors='black', linewidths=2, zorder=5, label='最优配置')
ax6.set_xlabel('Ki', fontsize=13, fontweight='bold')
ax6.set_ylabel('路径长度 (m)', fontsize=13, fontweight='bold')
ax6.set_title('Ki与路径长度关系', fontsize=14, fontweight='bold')
ax6.grid(True, linestyle='--', alpha=0.3)
ax6.legend(fontsize=10)
plt.colorbar(scatter2, ax=ax6, label='路径长度(m)')

# Kd vs 路径长度
ax7 = plt.subplot(2, 3, 3)
scatter3 = ax7.scatter(stage2_df['Kd'], stage2_df['avg_length'],
                      c=stage2_df['avg_length'], cmap='coolwarm',
                      s=scatter_size, alpha=scatter_alpha, edgecolors='black', linewidth=0.5)
ax7.scatter(optimal_kd, stage2_path[0], s=300, c='red', marker='*',
           edgecolors='black', linewidths=2, zorder=5, label='最优配置')
ax7.set_xlabel('Kd', fontsize=13, fontweight='bold')
ax7.set_ylabel('路径长度 (m)', fontsize=13, fontweight='bold')
ax7.set_title('Kd与路径长度关系', fontsize=14, fontweight='bold')
ax7.grid(True, linestyle='--', alpha=0.3)
ax7.legend(fontsize=10)
plt.colorbar(scatter3, ax=ax7, label='路径长度(m)')

# Kp vs Ki (颜色表示路径长度)
ax8 = plt.subplot(2, 3, 4)
scatter4 = ax8.scatter(stage2_df['Kp'], stage2_df['Ki'],
                      c=stage2_df['avg_length'], cmap='viridis',
                      s=scatter_size, alpha=scatter_alpha, edgecolors='black', linewidth=0.5)
ax8.scatter(optimal_kp, optimal_ki, s=300, c='red', marker='*',
           edgecolors='black', linewidths=2, zorder=5, label='最优配置')
ax8.set_xlabel('Kp', fontsize=13, fontweight='bold')
ax8.set_ylabel('Ki', fontsize=13, fontweight='bold')
ax8.set_title('Kp-Ki参数空间分布', fontsize=14, fontweight='bold')
ax8.grid(True, linestyle='--', alpha=0.3)
ax8.legend(fontsize=10)
plt.colorbar(scatter4, ax=ax8, label='路径长度(m)')

# Kp vs Kd (颜色表示路径长度)
ax9 = plt.subplot(2, 3, 5)
scatter5 = ax9.scatter(stage2_df['Kp'], stage2_df['Kd'],
                      c=stage2_df['avg_length'], cmap='viridis',
                      s=scatter_size, alpha=scatter_alpha, edgecolors='black', linewidth=0.5)
ax9.scatter(optimal_kp, optimal_kd, s=300, c='red', marker='*',
           edgecolors='black', linewidths=2, zorder=5, label='最优配置')
ax9.set_xlabel('Kp', fontsize=13, fontweight='bold')
ax9.set_ylabel('Kd', fontsize=13, fontweight='bold')
ax9.set_title('Kp-Kd参数空间分布', fontsize=14, fontweight='bold')
ax9.grid(True, linestyle='--', alpha=0.3)
ax9.legend(fontsize=10)
plt.colorbar(scatter5, ax=ax9, label='路径长度(m)')

# Ki vs Kd (颜色表示路径长度)
ax10 = plt.subplot(2, 3, 6)
scatter6 = ax10.scatter(stage2_df['Ki'], stage2_df['Kd'],
                       c=stage2_df['avg_length'], cmap='viridis',
                       s=scatter_size, alpha=scatter_alpha, edgecolors='black', linewidth=0.5)
ax10.scatter(optimal_ki, optimal_kd, s=300, c='red', marker='*',
            edgecolors='black', linewidths=2, zorder=5, label='最优配置')
ax10.set_xlabel('Ki', fontsize=13, fontweight='bold')
ax10.set_ylabel('Kd', fontsize=13, fontweight='bold')
ax10.set_title('Ki-Kd参数空间分布', fontsize=14, fontweight='bold')
ax10.grid(True, linestyle='--', alpha=0.3)
ax10.legend(fontsize=10)
plt.colorbar(scatter6, ax=ax10, label='路径长度(m)')

plt.tight_layout(rect=[0, 0, 1, 0.97])

output_file2 = Path("results/PID参数与路径长度相关性_真实数据.png")
plt.savefig(output_file2, dpi=300, bbox_inches='tight', facecolor='white')
print(f"✓ 图表2已保存: {output_file2}")
plt.close()

print("\n" + "="*80)
print("图表生成完成！")
print("="*80)
print(f"\n生成的文件:")
print(f"  1. {output_file}")
print(f"  2. {output_file2}")

if not stage3_available:
    print(f"\n⚠️  提示: Stage 3数据尚未完成")
    print(f"   等待 run_stage3_validation.py 完成后重新运行本脚本以更新图表")

print("\n" + "="*80 + "\n")
