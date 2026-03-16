% =========================================================================
%               SCI论文标准 - 算法性能对比实验
% =========================================================================
% 
% 符合SCI论文对纯仿真算法对比实验的标准要求:
%   1. 每种配置运行20次以上，保证统计显著性
%   2. 报告 Mean ± Std 格式
%   3. 包含6种标准性能指标
%   4. 2D和3D多维度测试
%   5. 生成可直接写入论文的对比表
%
% SCI论文标准6指标:
%   (1) Path Length (mm)     - 路径质量核心指标
%   (2) Planning Time (s)    - 算法效率指标
%   (3) Success Rate (%)     - 算法可靠性指标
%   (4) Tree Nodes           - 计算开销指标
%   (5) Smoothness (rad)     - 路径平滑度 (角度变化标准差)
%   (6) Path Efficiency      - 路径效率 (直线距离/实际路径长度)
%
% 对比算法 (6种):
%   RRT, RRT-Connect, RRT*, Informed RRT*, SC-RRT, Dynamic-RRT
%
% 使用方法:
%   直接运行此脚本即可启动完整SCI实验
% =========================================================================

clc; clear; close all;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║       SCI论文标准 - 路径规划算法对比实验 (6 Algorithms)       ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% ========== 实验参数配置 ==========
% 可根据需要调整以下参数

NUM_RUNS = 500;           % <<<<<<<<<< 在这里修改每组实验的运行次数 (建议 ≥20)

% --- 2D实验参数 (论文Table 1) ---
RUN_2D = true;
NUM_OBS_2D = 225;        % 2D障碍物数量
MAX_ITER_2D = 5000;      % 2D最大迭代次数

% --- 3D实验参数 (论文Table 1) ---
RUN_3D = true;
NUM_OBS_3D = 400;        % 3D障碍物数量
MAX_ITER_3D = 8000;      % 3D最大迭代次数

fprintf('实验配置:\n');
fprintf('  运行次数: %d 次/算法/维度\n', NUM_RUNS);
fprintf('  2D环境: %dx%d, %d个障碍物, 最大%d次迭代\n', 1500, 1500, NUM_OBS_2D, MAX_ITER_2D);
fprintf('  3D环境: %d^3, %d个障碍物, 最大%d次迭代\n', 1500, NUM_OBS_3D, MAX_ITER_3D);
fprintf('\n');

%% ========== 路径设置 ==========
script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
addpath(fullfile(script_dir, 'sc_rrt'));
addpath(fullfile(script_dir, 'dynamic_rrt'));

OUTPUT_DIR = fullfile(script_dir, 'results');
if ~exist(OUTPUT_DIR, 'dir'), mkdir(OUTPUT_DIR); end

%% ========== 启动实验 ==========
% 注: 使用 batchCompareAlgorithms 保证每次运行所有算法在相同环境下测试(公平对比)
%     输出: CSV(摘要统计+逐次原始数据) + MAT + PNG 图表
%     CSV 可直接导入 Python/MATLAB 绘制柱状图、箱线图、折线图

total_start = tic;
all_files   = struct();

% ─── 2D 实验 ────────────────────────────────────────────────────────────────
if RUN_2D
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('【2D 实验】%d 次运行 × 6 算法 (障碍物:%d, 最大迭代:%d)\n', ...
            NUM_RUNS, NUM_OBS_2D, MAX_ITER_2D);
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

    result_2d = batchCompareAlgorithms(NUM_RUNS, MAX_ITER_2D, ...
        'Dimension',    '2D', ...
        'NumObstacles', NUM_OBS_2D, ...
        'SaveResults',  true, ...
        'Visualize',    false, ...
        'OutputDir',    OUTPUT_DIR);

    if isfield(result_2d, 'files')
        all_files.d2 = result_2d.files;
    end
    fprintf('✓ 2D 实验全部完成\n\n');
end

% ─── 3D 实验 ────────────────────────────────────────────────────────────────
if RUN_3D
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('【3D 实验】%d 次运行 × 6 算法 (障碍物:%d, 最大迭代:%d)\n', ...
            NUM_RUNS, NUM_OBS_3D, MAX_ITER_3D);
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

    result_3d = batchCompareAlgorithms(NUM_RUNS, MAX_ITER_3D, ...
        'Dimension',    '3D', ...
        'NumObstacles', NUM_OBS_3D, ...
        'SaveResults',  true, ...
        'Visualize',    false, ...
        'OutputDir',    OUTPUT_DIR);

    if isfield(result_3d, 'files')
        all_files.d3 = result_3d.files;
    end
    fprintf('✓ 3D 实验全部完成\n\n');
end

%% ========== 完成总结 ==========
total_time = toc(total_start);
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                  SCI 实验全部完成                             ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('总耗时: %.1f 秒 (%.1f 分钟)\n', total_time, total_time / 60);
fprintf('\n');

%% ========== 核心指标汇总表 (4项) ==========
% 格式: 规划时间 / 路径长度 / 平滑度 / 节点数  (Mean ± Std)

if RUN_2D && exist('result_2d','var')
    print_summary_table(result_2d.statistics, result_2d.algorithms, '2D', NUM_RUNS);
end
if RUN_3D && exist('result_3d','var')
    print_summary_table(result_3d.statistics, result_3d.algorithms, '3D', NUM_RUNS);
end

fprintf('\n实验全部完成。可直接运行 run_SCI_experiment.m 重新执行。\n\n');

%% ========== 本地辅助函数 ==========

function print_summary_table(statistics, algorithms, dim_str, num_runs)
% 在命令窗口打印4项核心指标汇总表（规划时间/路径长度/平滑度/节点数）
    algo_labels = cellfun(@(x) strrep(x,'_Basic',''), algorithms, 'UniformOutput', false);
    algo_labels = cellfun(@(x) strrep(x,'_','-'),  algo_labels,  'UniformOutput', false);

    W = 100;
    fprintf('\n');
    fprintf('%s\n', repmat('═', 1, W));
    fprintf('  核心指标汇总 — %s 环境  (N=%d 次, 格式: Mean ± Std)\n', dim_str, num_runs);
    fprintf('%s\n', repmat('═', 1, W));
    fprintf('  %-24s  %-18s  %-18s  %-16s  %-12s\n', ...
        '算法', '规划时间 (s)', '路径长度 (mm)', '路径平滑度 (rad)', '节点数');
    fprintf('  %s\n', repmat('─', 1, W-2));
    for i = 1:length(algorithms)
        a  = algorithms{i};
        lb = algo_labels{i};
        if ~isfield(statistics, a), continue; end
        s  = statistics.(a);
        pt  = get_ms(s, 'time_mean',        'time_std',        '%.4f');
        pl  = get_ms(s, 'path_length_mean', 'path_length_std', '%.2f');
        sm  = get_ms(s, 'smoothness_mean',  'smoothness_std',  '%.4f');
        nd  = get_ms(s, 'nodes_mean',       'nodes_std',       '%.0f');
        fprintf('  %-24s  %-18s  %-18s  %-16s  %s\n', lb, pt, pl, sm, nd);
    end
    fprintf('%s\n\n', repmat('═', 1, W));
end

function s = get_ms(st, mf, sf, fmt)
    if isfield(st,mf) && ~isnan(st.(mf))
        if isfield(st,sf) && ~isnan(st.(sf))
            s = sprintf([fmt ' ± ' fmt], st.(mf), st.(sf));
        else
            s = sprintf(fmt, st.(mf));
        end
    else
        s = 'N/A';
    end
end
