import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，避免GUI显示问题
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def setup_plot_style():
    """设置专业的图表样式"""
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']  # 英文期刊适配字体
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['axes.linewidth'] = 1.2  # 坐标轴线条宽度
    plt.rcParams['grid.alpha'] = 0.3      # 网格透明度
    plt.rcParams['legend.frameon'] = True # 图例边框
    plt.rcParams['legend.framealpha'] = 0.9 # 图例透明度
    plt.rcParams['xtick.direction'] = 'in' # x轴刻度向内
    plt.rcParams['ytick.direction'] = 'in' # y轴刻度向内

def extract_mean(value_str):
    """从 '均值±标准差' 格式中提取均值"""
    if pd.isna(value_str) or value_str == '' or not isinstance(value_str, str):
        return 0.0
    if '+-' in value_str:
        return float(value_str.split('+-')[0].strip())
    try:
        return float(value_str.strip())
    except:
        return 0.0

def load_rrt_data(file_path, skip_comment_lines=16):
    """加载并处理 RRT 算法 CSV 数据"""
    # 手动读取数据（处理注释行和格式问题）
    data = []
    headers = None
    
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        
        # 找到表头（以"算法,"开头）
        for i, line in enumerate(lines):
            line_stripped = line.strip()
            if line_stripped.startswith('算法,'):
                headers = line_stripped.split(',')
                # 读取后续数据行（跳过注释行）
                for line_data in lines[i+1:]:
                    line_data_stripped = line_data.strip()
                    if line_data_stripped == '' or line_data_stripped.startswith('#'):
                        continue
                    # 分割数据（处理可能的多余逗号）
                    parts = line_data_stripped.split(',')
                    if len(parts) >= len(headers):
                        data.append(parts[:len(headers)])
                break
    
    # 创建DataFrame并处理
    df = pd.DataFrame(data, columns=headers)
    # 提取主要算法（前6行：RRT、RRT-Connect、RRT*、Informed-RRT*、SC-RRT、Dynamic-RRT）
    main_df = df.iloc[:6].copy()
    
    # 处理关键指标
    key_metrics = ['路径长度(mm)', '收敛时间(s)', '规划时间(s)', '树节点数', '平滑度(rad)', '路径效率']
    for metric in key_metrics:
        if metric in main_df.columns:
            main_df[metric] = main_df[metric].apply(extract_mean)
    
    # 简化算法名称
    name_mapping = {
        'RRT_Basic': 'RRT',
        'RRT_Connect_Basic': 'RRT-Connect',
        'RRT_Star_Basic': 'RRT*',
        'Informed_RRT_Star_Basic': 'Informed-RRT*',
        'SC_RRT_Basic': 'SC-RRT',
        'Dynamic_RRT_Basic': 'Dynamic-RRT'
    }
    main_df['算法简化名'] = main_df['算法'].map(name_mapping)
    
    return main_df

def create_comparison_plots(df_2d, df_3d, save_dir='./'):
    """
    创建2D vs 3D 综合对比图
    
    参数:
    df_2d: 2D场景数据DataFrame（load_rrt_data返回结果）
    df_3d: 3D场景数据DataFrame（load_rrt_data返回结果）
    save_dir: 图片保存目录
    """
    setup_plot_style()
    
    # 颜色和标记配置
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    markers = ['o', 's', '^', 'D', 'v', '<']
    
    # 创建综合对比图（2x2子图）
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Performance Comparison: SC-RRT vs Classic RRT-based Algorithms', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    # 1. 路径长度对比
    ax1.plot(df_2d['算法简化名'], df_2d['路径长度(mm)'], 
             marker=markers[0], color=colors[0], linewidth=2.5, markersize=8, label='2D Scene')
    ax1.plot(df_3d['算法简化名'], df_3d['路径长度(mm)'], 
             marker=markers[1], color=colors[1], linewidth=2.5, markersize=8, label='3D Scene')
    ax1.set_title('Path Length Comparison (mm)', fontsize=14, fontweight='bold', pad=15)
    ax1.set_xlabel('Algorithms', fontsize=12)
    ax1.set_ylabel('Path Length (mm)', fontsize=12)
    ax1.legend(fontsize=10)
    ax1.grid(True)
    ax1.tick_params(axis='x', rotation=45)
    
    # 2. 收敛时间对比
    ax2.plot(df_2d['算法简化名'], df_2d['收敛时间(s)'], 
             marker=markers[2], color=colors[2], linewidth=2.5, markersize=8, label='2D Scene')
    ax2.plot(df_3d['算法简化名'], df_3d['收敛时间(s)'], 
             marker=markers[3], color=colors[3], linewidth=2.5, markersize=8, label='3D Scene')
    ax2.set_title('Convergence Time Comparison (s)', fontsize=14, fontweight='bold', pad=15)
    ax2.set_xlabel('Algorithms', fontsize=12)
    ax2.set_ylabel('Convergence Time (s)', fontsize=12)
    ax2.legend(fontsize=10)
    ax2.grid(True)
    ax2.tick_params(axis='x', rotation=45)
    
    # 3. 树节点数对比
    ax3.plot(df_2d['算法简化名'], df_2d['树节点数'], 
             marker=markers[4], color=colors[4], linewidth=2.5, markersize=8, label='2D Scene')
    ax3.plot(df_3d['算法简化名'], df_3d['树节点数'], 
             marker=markers[5], color=colors[5], linewidth=2.5, markersize=8, label='3D Scene')
    ax3.set_title('Tree Nodes Comparison', fontsize=14, fontweight='bold', pad=15)
    ax3.set_xlabel('Algorithms', fontsize=12)
    ax3.set_ylabel('Number of Tree Nodes', fontsize=12)
    ax3.legend(fontsize=10)
    ax3.grid(True)
    ax3.tick_params(axis='x', rotation=45)
    
    # 4. 路径效率对比
    ax4.plot(df_2d['算法简化名'], df_2d['路径效率'], 
             marker=markers[0], color=colors[0], linewidth=2.5, markersize=8, label='2D Scene')
    ax4.plot(df_3d['算法简化名'], df_3d['路径效率'], 
             marker=markers[1], color=colors[1], linewidth=2.5, markersize=8, label='3D Scene')
    ax4.set_title('Path Efficiency Comparison', fontsize=14, fontweight='bold', pad=15)
    ax4.set_xlabel('Algorithms', fontsize=12)
    ax4.set_ylabel('Path Efficiency', fontsize=12)
    ax4.set_ylim(0.7, 1.0)  # 固定y轴范围，突出差异
    ax4.legend(fontsize=10)
    ax4.grid(True)
    ax4.tick_params(axis='x', rotation=45)
    
    # 保存图片
    plt.tight_layout()
    plt.subplots_adjust(top=0.93)
    save_path = Path(save_dir) / 'SC_RRT_Comprehensive_Comparison.png'
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print(f"综合对比图已保存: {save_path}")

def create_single_scene_plot(df, scene_type='2D', save_dir='./'):
    """
    创建单个场景（2D/3D）的详细对比图
    
    参数:
    df: 场景数据DataFrame（load_rrt_data返回结果）
    scene_type: 场景类型（'2D' 或 '3D'）
    save_dir: 图片保存目录
    """
    setup_plot_style()
    
    # 创建2x2子图
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Performance Comparison in {scene_type} Scene', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    # 找到SC-RRT的索引（用于突出显示）
    sc_rrt_idx = df[df['算法简化名'] == 'SC-RRT'].index[0]
    
    # 1. 路径长度
    axs[0,0].plot(df['算法简化名'], df['路径长度(mm)'], 
                  marker='o', color='#1f77b4', linewidth=3, markersize=10,
                  markerfacecolor='white', markeredgewidth=2)
    axs[0,0].scatter(df.loc[sc_rrt_idx, '算法简化名'], df.loc[sc_rrt_idx, '路径长度(mm)'],
                     color='red', s=200, zorder=5, edgecolor='black', linewidth=2, label='SC-RRT')
    axs[0,0].set_title('Path Length (mm)', fontsize=13, fontweight='bold', pad=12)
    axs[0,0].set_xlabel('Algorithms', fontsize=11)
    axs[0,0].set_ylabel('Path Length (mm)', fontsize=11)
    axs[0,0].grid(True)
    axs[0,0].tick_params(axis='x', rotation=45)
    axs[0,0].legend(fontsize=10)
    
    # 2. 收敛时间
    axs[0,1].plot(df['算法简化名'], df['收敛时间(s)'], 
                  marker='s', color='#ff7f0e', linewidth=3, markersize=10,
                  markerfacecolor='white', markeredgewidth=2)
    axs[0,1].scatter(df.loc[sc_rrt_idx, '算法简化名'], df.loc[sc_rrt_idx, '收敛时间(s)'],
                     color='red', s=200, zorder=5, edgecolor='black', linewidth=2, label='SC-RRT')
    axs[0,1].set_title('Convergence Time (s)', fontsize=13, fontweight='bold', pad=12)
    axs[0,1].set_xlabel('Algorithms', fontsize=11)
    axs[0,1].set_ylabel('Convergence Time (s)', fontsize=11)
    axs[0,1].grid(True)
    axs[0,1].tick_params(axis='x', rotation=45)
    axs[0,1].legend(fontsize=10)
    
    # 3. 树节点数
    axs[1,0].plot(df['算法简化名'], df['树节点数'], 
                  marker='^', color='#2ca02c', linewidth=3, markersize=10,
                  markerfacecolor='white', markeredgewidth=2)
    axs[1,0].scatter(df.loc[sc_rrt_idx, '算法简化名'], df.loc[sc_rrt_idx, '树节点数'],
                     color='red', s=200, zorder=5, edgecolor='black', linewidth=2, label='SC-RRT')
    axs[1,0].set_title('Number of Tree Nodes', fontsize=13, fontweight='bold', pad=12)
    axs[1,0].set_xlabel('Algorithms', fontsize=11)
    axs[1,0].set_ylabel('Tree Nodes', fontsize=11)
    axs[1,0].grid(True)
    axs[1,0].tick_params(axis='x', rotation=45)
    axs[1,0].legend(fontsize=10)
    
    # 4. 路径效率
    axs[1,1].plot(df['算法简化名'], df['路径效率'], 
                  marker='D', color='#d62728', linewidth=3, markersize=10,
                  markerfacecolor='white', markeredgewidth=2)
    axs[1,1].scatter(df.loc[sc_rrt_idx, '算法简化名'], df.loc[sc_rrt_idx, '路径效率'],
                     color='red', s=200, zorder=5, edgecolor='black', linewidth=2, label='SC-RRT')
    axs[1,1].set_title('Path Efficiency', fontsize=13, fontweight='bold', pad=12)
    axs[1,1].set_xlabel('Algorithms', fontsize=11)
    axs[1,1].set_ylabel('Path Efficiency', fontsize=11)
    axs[1,1].set_ylim(0.75, 1.0)  # 固定y轴范围
    axs[1,1].grid(True)
    axs[1,1].tick_params(axis='x', rotation=45)
    axs[1,1].legend(fontsize=10)
    
    # 保存图片
    plt.tight_layout()
    plt.subplots_adjust(top=0.93)
    save_path = Path(save_dir) / f'SC_RRT_{scene_type}_Detailed_Comparison.png'
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()
    print(f"{scene_type}场景详细图已保存: {save_path}")

def create_data_summary(df_2d, df_3d, save_dir='./'):
    """创建2D和3D数据汇总表（CSV格式）"""
    # 提取关键数据
    summary_data = {
        'Algorithm': df_2d['算法简化名'].tolist(),
        '2D_Path_Length_mm': df_2d['路径长度(mm)'].tolist(),
        '3D_Path_Length_mm': df_3d['路径长度(mm)'].tolist(),
        '2D_Convergence_Time_s': df_2d['收敛时间(s)'].tolist(),
        '3D_Convergence_Time_s': df_3d['收敛时间(s)'].tolist(),
        '2D_Tree_Nodes': df_2d['树节点数'].tolist(),
        '3D_Tree_Nodes': df_3d['树节点数'].tolist(),
        '2D_Path_Efficiency': df_2d['路径效率'].tolist(),
        '3D_Path_Efficiency': df_3d['路径效率'].tolist()
    }
    
    summary_df = pd.DataFrame(summary_data)
    save_path = Path(save_dir) / 'SC_RRT_Performance_Summary.csv'
    summary_df.to_csv(save_path, index=False)
    print(f"数据汇总表已保存: {save_path}")

# ------------------- 主程序入口 -------------------
if __name__ == "__main__":
    # 1. 配置文件路径（下次使用时只需修改这两个路径）
    csv_path_2d = r'Dynamic-rrt算法与SC-RRT算法对比实验\四算法对比实验1-11针对sc-rrt算法的四算法对比实验完善\results\comparison_2D_20260315_000618.csv'  # 你的2D数据CSV路径
    csv_path_3d = r'Dynamic-rrt算法与SC-RRT算法对比实验\四算法对比实验1-11针对sc-rrt算法的四算法对比实验完善\results\comparison_3D_20260315_034219.csv'  # 你的3D数据CSV路径
    # csv_path_3d = r'Dynamic-rrt算法与SC-RRT算法对比实验\四算法对比实验1-10针对sc-rrt算法的鲁棒性实验完善\results\comparison_3D_20260314_174841.csv'  # 你的3D数据CSV路径
    save_directory = './'  # 图片保存目录（默认当前目录）
    
    # 2. 加载和处理数据
    print("正在加载数据...")
    df_2d_data = load_rrt_data(csv_path_2d)
    df_3d_data = load_rrt_data(csv_path_3d)
    print("数据加载完成！")
    
    # 3. 生成图表
    print("\n正在生成综合对比图...")
    create_comparison_plots(df_2d_data, df_3d_data, save_directory)
    
    print("\n正在生成2D场景详细图...")
    create_single_scene_plot(df_2d_data, '2D', save_directory)
    
    print("\n正在生成3D场景详细图...")
    create_single_scene_plot(df_3d_data, '3D', save_directory)
    
    # 4. 生成数据汇总表
    print("\n正在生成数据汇总表...")
    create_data_summary(df_2d_data, df_3d_data, save_directory)
    
    print("\n所有文件生成完成！")