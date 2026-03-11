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

NUM_RUNS = 20;           % 每种配置运行次数 (SCI标准: ≥20)

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

%% ========== 启动实验 ==========
total_start = tic;

if RUN_2D && RUN_3D
    dim_mode = 'both';
elseif RUN_2D
    dim_mode = '2D';
else
    dim_mode = '3D';
end

fprintf('>> 正在启动 %d 次运行的 %s 实验...\n\n', NUM_RUNS, dim_mode);

runAlgorithmComparison(...
    'NumRuns', NUM_RUNS, ...
    'Dimension', dim_mode, ...
    'NumObstacles2D', NUM_OBS_2D, ...
    'NumObstacles3D', NUM_OBS_3D, ...
    'MaxIterations2D', MAX_ITER_2D, ...
    'MaxIterations3D', MAX_ITER_3D, ...
    'SaveResults', true, ...
    'Visualize', true);

total_time = toc(total_start);
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                  SCI实验全部完成                              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('总耗时: %.1f 秒 (%.1f 分钟)\n', total_time, total_time/60);
fprintf('结果已保存至 results/ 目录\n\n');
