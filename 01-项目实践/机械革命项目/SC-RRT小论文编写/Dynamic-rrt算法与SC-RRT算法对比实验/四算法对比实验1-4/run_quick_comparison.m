% =========================================================================
%     五算法对比实验 - 一键运行脚本 (2D+3D 合并窗口版)
% =========================================================================
% 功能: 一键对比5种RRT算法变体的性能
%   同时运行2D和3D环境，结果分别放入两个窗口
%
% 支持算法:
%   1. RRT_Basic          - 原生RRT算法
%   2. RRT_Connect_Basic  - RRT-Connect算法 (双向快速连接)
%   3. RRT_Star_Basic     - RRT*算法 (渐进最优)
%   4. SC_RRT_Basic       - SC-RRT算法 (双向+PID+Pareto)
%   5. Dynamic_RRT_Basic  - Dynamic-RRT算法 (Informed+Pareto动态规划)
%
% 输出:
%   - 窗口1: 2D环境下所有算法的路径规划结果 (subplot)
%   - 窗口2: 3D环境下所有算法的路径规划结果 (subplot)
%   - 窗口3: 性能指标柱状图对比
%   - 控制台输出完整统计表
%
% 使用方法:
%   直接在MATLAB中运行本脚本即可
% =========================================================================
clear; clc; close all;

%% =================== 环境配置 ===================
fprintf('\n');
fprintf('================================================================\n');
fprintf('           五算法对比实验 - 2D & 3D 合并窗口\n');
fprintf('================================================================\n\n');

% ===== 用户可修改区域 =====
config.bounds_2d   = [0 1500 0 1500];                 % 2D空间边界
config.bounds_3d   = [0 1500 0 1500 0 1500];           % 3D空间边界
config.start_2d    = [400 400];                        % 2D起点
config.goal_2d     = [1100 1100];                      % 2D终点
config.start_3d    = [200 200 200];                    % 3D起点
config.goal_3d     = [1300 1300 1300];                 % 3D终点
config.num_obs_2d  = 225;                              % 2D障碍物数量
config.num_obs_3d  = 400;                              % 3D障碍物数量
config.obs_radius  = 15;                               % 障碍物基准半径
config.max_iter_2d = 5000;                             % 2D最大迭代次数
config.max_iter_3d = 8000;                             % 3D最大迭代次数
config.seed        = 42;                               % 随机种子
% ===========================

%% =================== 设置路径 ===================
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
addpath(thisDir);
addpath(fullfile(thisDir, 'sc_rrt'));
addpath(fullfile(thisDir, 'dynamic_rrt'));
cd(thisDir);

fprintf('工作目录: %s\n', thisDir);

%% =================== 定义算法 ===================
algorithms = {
    struct('name', 'RRT',          'func', @(e,n) run_rrt(e,n),          'color', [0.85 0.33 0.10]);
    struct('name', 'RRT-Connect',  'func', @(e,n) run_rrt_connect(e,n),  'color', [0.93 0.69 0.13]);
    struct('name', 'RRT*',         'func', @(e,n) run_rrt_star(e,n),     'color', [0.47 0.67 0.19]);
    struct('name', 'SC-RRT',       'func', @(e,n) run_sc_rrt(e,n),       'color', [0.00 0.45 0.74]);
    struct('name', 'Dynamic-RRT',  'func', @(e,n) run_dynamic_rrt(e,n),  'color', [1.00 0.00 0.00]);
};
num_alg = length(algorithms);

% 计算subplot布局
if num_alg <= 4
    nrows = 2; ncols = 2;
elseif num_alg <= 6
    nrows = 2; ncols = 3;
else
    nrows = 3; ncols = 3;
end

%% =================================================================
%%                         2D 环境测试
%% =================================================================
fprintf('\n================================================================\n');
fprintf('【2D 环境测试】\n');
fprintf('================================================================\n\n');

% 生成2D环境
if ischar(config.seed) && strcmp(config.seed, 'shuffle')
    rng('shuffle');
else
    rng(config.seed);
end

fprintf('正在生成2D障碍物环境 (%d个障碍物)...\n', config.num_obs_2d);
opts_2d = struct();
opts_2d.start_point = config.start_2d;
opts_2d.goal_point  = config.goal_2d;
opts_2d.fixed_radius = config.obs_radius;
opts_2d.min_spacing = 5.0;
opts_2d.clearance   = 45.0;
env_2d = EnvironmentConfig.generate2DEnvironment(config.bounds_2d, config.num_obs_2d, opts_2d);
fprintf('✓ 2D环境生成完成: %d个障碍物\n\n', env_2d.num);

% 运行所有算法
results_2d = cell(num_alg, 1);

for idx = 1:num_alg
    alg = algorithms{idx};
    fprintf('【%d/%d】 %s (2D) 运行中...\n', idx, num_alg, alg.name);
    
    if isnumeric(config.seed)
        rng(config.seed + 100 + idx);
    end
    
    try
        tic;
        [path, tree, success, metrics] = alg.func(env_2d, config.max_iter_2d);
        elapsed = toc;
        
        if success && ~isempty(path)
            pathLen = sum(vecnorm(diff(path), 2, 2));
            smooth_val = calcSmoothness(path);
            nodeCount = getNodeCount(tree);
            
            results_2d{idx} = struct('success', true, 'path', path, 'tree', tree, ...
                                     'time', elapsed, 'length', pathLen, ...
                                     'nodes', nodeCount, 'smoothness', smooth_val);
            
            fprintf('  >> 成功! Time:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f\n', ...
                    elapsed, pathLen, nodeCount, smooth_val);
        else
            results_2d{idx} = struct('success', false, 'time', elapsed);
            fprintf('  >> 失败 (未找到路径)\n');
        end
    catch ME
        results_2d{idx} = struct('success', false, 'time', 0);
        fprintf('  >> 错误: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('     位置: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
        end
    end
    fprintf('\n');
end

%% =================================================================
%%                         3D 环境测试
%% =================================================================
fprintf('\n================================================================\n');
fprintf('【3D 环境测试】\n');
fprintf('================================================================\n\n');

% 生成3D环境
if isnumeric(config.seed)
    rng(config.seed + 200);
end

fprintf('正在生成3D障碍物环境 (%d个障碍物)...\n', config.num_obs_3d);
opts_3d = struct();
opts_3d.start_point = config.start_3d;
opts_3d.goal_point  = config.goal_3d;
opts_3d.fixed_radius = config.obs_radius;
opts_3d.min_spacing = 5.0;
opts_3d.clearance   = 45.0;
env_3d = EnvironmentConfig.generate3DEnvironment(config.bounds_3d, config.num_obs_3d, opts_3d);
fprintf('✓ 3D环境生成完成: %d个障碍物\n\n', env_3d.num);

% 运行所有算法
results_3d = cell(num_alg, 1);

for idx = 1:num_alg
    alg = algorithms{idx};
    fprintf('【%d/%d】 %s (3D) 运行中...\n', idx, num_alg, alg.name);
    
    if isnumeric(config.seed)
        rng(config.seed + 300 + idx);
    end
    
    try
        tic;
        [path, tree, success, metrics] = alg.func(env_3d, config.max_iter_3d);
        elapsed = toc;
        
        if success && ~isempty(path)
            pathLen = sum(vecnorm(diff(path), 2, 2));
            smooth_val = calcSmoothness(path);
            nodeCount = getNodeCount(tree);
            
            results_3d{idx} = struct('success', true, 'path', path, 'tree', tree, ...
                                     'time', elapsed, 'length', pathLen, ...
                                     'nodes', nodeCount, 'smoothness', smooth_val);
            
            fprintf('  >> 成功! Time:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f\n', ...
                    elapsed, pathLen, nodeCount, smooth_val);
        else
            results_3d{idx} = struct('success', false, 'time', elapsed);
            fprintf('  >> 失败 (未找到路径)\n');
        end
    catch ME
        results_3d{idx} = struct('success', false, 'time', 0);
        fprintf('  >> 错误: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('     位置: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
        end
    end
    fprintf('\n');
end

%% =================================================================
%%       窗口1: 2D 路径规划结果 (所有算法放在同一窗口)
%% =================================================================
fprintf('================================================================\n');
fprintf('生成结果图...\n');
fprintf('================================================================\n\n');

fig_2d = figure('Name', '2D 路径规划结果对比', ...
                'Position', [30 80 ncols*480 nrows*420], 'Color', 'w');

for idx = 1:num_alg
    alg = algorithms{idx};
    res = results_2d{idx};
    
    subplot(nrows, ncols, idx);
    hold on; axis equal; grid on;
    xlim(config.bounds_2d(1:2)); ylim(config.bounds_2d(3:4));
    xlabel('X (mm)', 'FontSize', 10);
    ylabel('Y (mm)', 'FontSize', 10);
    
    % 绘制障碍物
    for i = 1:env_2d.num
        obs = env_2d.obstacles(i, :);
        rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
                 'Curvature', [1 1], 'FaceColor', [0.15 0.15 0.15], ...
                 'EdgeColor', 'none');
    end
    
    % 绘制起终点
    plot(config.start_2d(1), config.start_2d(2), 'go', ...
         'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
    plot(config.goal_2d(1), config.goal_2d(2), 'rs', ...
         'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    
    % 绘制路径 + 标题
    if res.success
        path = res.path;
        plot(path(:,1), path(:,2), 'Color', alg.color, 'LineWidth', 2.5);
        title_str = sprintf('%s\nTime:%.3fs | Len:%.1fmm | Nodes:%d | Smooth:%.4f', ...
                           alg.name, res.time, res.length, res.nodes, res.smoothness);
        title(title_str, 'FontSize', 11, 'FontWeight', 'bold');
    else
        title(sprintf('%s\n(规划失败)', alg.name), 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');
    end
    
    set(gca, 'FontSize', 9);
end

sgtitle('2D 路径规划结果对比', 'FontSize', 16, 'FontWeight', 'bold');
drawnow;

%% =================================================================
%%       窗口2: 3D 路径规划结果 (所有算法放在同一窗口)
%% =================================================================
fig_3d = figure('Name', '3D 路径规划结果对比', ...
                'Position', [60 50 ncols*480 nrows*420], 'Color', 'w');

for idx = 1:num_alg
    alg = algorithms{idx};
    res = results_3d{idx};
    
    subplot(nrows, ncols, idx);
    hold on; axis equal; grid on;
    view(45, 30);
    xlim(config.bounds_3d(1:2));
    ylim(config.bounds_3d(3:4));
    zlim(config.bounds_3d(5:6));
    xlabel('X', 'FontSize', 9);
    ylabel('Y', 'FontSize', 9);
    zlabel('Z', 'FontSize', 9);
    
    % 绘制障碍物 (降低球体精度以加快渲染)
    [Xs, Ys, Zs] = sphere(8);
    for i = 1:env_3d.num
        surf(Xs*env_3d.obstacles(i,4)+env_3d.obstacles(i,1), ...
             Ys*env_3d.obstacles(i,4)+env_3d.obstacles(i,2), ...
             Zs*env_3d.obstacles(i,4)+env_3d.obstacles(i,3), ...
             'FaceColor', [0 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end
    
    % 绘制起终点
    plot3(config.start_3d(1), config.start_3d(2), config.start_3d(3), 'go', ...
          'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
    plot3(config.goal_3d(1), config.goal_3d(2), config.goal_3d(3), 'rs', ...
          'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    
    % 绘制路径 + 标题
    if res.success
        path = res.path;
        if size(path, 2) >= 3
            plot3(path(:,1), path(:,2), path(:,3), 'Color', alg.color, 'LineWidth', 2.5);
        end
        title_str = sprintf('%s\nTime:%.3fs | Len:%.1fmm | Nodes:%d | Smooth:%.4f', ...
                           alg.name, res.time, res.length, res.nodes, res.smoothness);
        title(title_str, 'FontSize', 11, 'FontWeight', 'bold');
    else
        title(sprintf('%s\n(规划失败)', alg.name), 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');
    end
    
    lighting gouraud;
    camlight('headlight');
    set(gca, 'FontSize', 9);
end

sgtitle('3D 路径规划结果对比', 'FontSize', 16, 'FontWeight', 'bold');
drawnow;

%% =================================================================
%%       窗口3: 性能指标柱状图对比 (2D和3D合并)
%% =================================================================
fig_bar = figure('Name', '性能指标对比', 'Position', [100 100 1400 600], 'Color', 'w');

metric_names = {'计算时间 (s)', '路径长度 (mm)', '搜索节点数', '平滑度'};

for dim_idx = 1:2
    if dim_idx == 1
        results_cur = results_2d;
        dim_label = '2D';
    else
        results_cur = results_3d;
        dim_label = '3D';
    end
    
    % 收集数据
    alg_names = {};
    times_arr = []; lengths_arr = []; nodes_arr = []; smooths_arr = [];
    colors_bar = [];
    
    for idx = 1:num_alg
        alg_names{end+1} = algorithms{idx}.name; %#ok<SAGROW>
        if results_cur{idx}.success
            times_arr(end+1)   = results_cur{idx}.time; %#ok<SAGROW>
            lengths_arr(end+1) = results_cur{idx}.length; %#ok<SAGROW>
            nodes_arr(end+1)   = results_cur{idx}.nodes; %#ok<SAGROW>
            smooths_arr(end+1) = results_cur{idx}.smoothness; %#ok<SAGROW>
        else
            times_arr(end+1)   = 0; %#ok<SAGROW>
            lengths_arr(end+1) = 0; %#ok<SAGROW>
            nodes_arr(end+1)   = 0; %#ok<SAGROW>
            smooths_arr(end+1) = 0; %#ok<SAGROW>
        end
        colors_bar = [colors_bar; algorithms{idx}.color]; %#ok<AGROW>
    end
    
    data_arrays = {times_arr, lengths_arr, nodes_arr, smooths_arr};
    
    for m = 1:4
        subplot(2, 4, (dim_idx-1)*4 + m);
        b = bar(data_arrays{m});
        b.FaceColor = 'flat';
        for k = 1:length(data_arrays{m})
            b.CData(k,:) = colors_bar(k,:);
        end
        set(gca, 'XTickLabel', alg_names, 'XTickLabelRotation', 25, 'FontSize', 8);
        ylabel(metric_names{m}, 'FontSize', 9);
        if m == 1
            title(sprintf('%s - %s', dim_label, metric_names{m}), 'FontSize', 10, 'FontWeight', 'bold');
        else
            title(metric_names{m}, 'FontSize', 10, 'FontWeight', 'bold');
        end
        grid on;
        
        % 标记失败的算法
        for idx = 1:num_alg
            if ~results_cur{idx}.success
                text(idx, 0, '失败', 'HorizontalAlignment', 'center', ...
                     'VerticalAlignment', 'bottom', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
            end
        end
    end
end

sgtitle('性能指标对比 (上: 2D, 下: 3D)', 'FontSize', 14, 'FontWeight', 'bold');
drawnow;

%% =================== 控制台统计表 ===================
fprintf('\n================================================================\n');
fprintf('                    性能统计表\n');
fprintf('================================================================\n\n');

fprintf('【2D 环境】\n');
fprintf('%-16s  %10s  %12s  %8s  %12s\n', '算法', '时间(s)', '路径长度(mm)', '节点数', '平滑度');
fprintf('----------------------------------------------------------------\n');
for idx = 1:num_alg
    if results_2d{idx}.success
        fprintf('%-16s  %10.4f  %12.2f  %8d  %12.4f\n', ...
                algorithms{idx}.name, results_2d{idx}.time, results_2d{idx}.length, ...
                results_2d{idx}.nodes, results_2d{idx}.smoothness);
    else
        fprintf('%-16s  %10s  %12s  %8s  %12s\n', algorithms{idx}.name, '失败', '-', '-', '-');
    end
end

fprintf('\n【3D 环境】\n');
fprintf('%-16s  %10s  %12s  %8s  %12s\n', '算法', '时间(s)', '路径长度(mm)', '节点数', '平滑度');
fprintf('----------------------------------------------------------------\n');
for idx = 1:num_alg
    if results_3d{idx}.success
        fprintf('%-16s  %10.4f  %12.2f  %8d  %12.4f\n', ...
                algorithms{idx}.name, results_3d{idx}.time, results_3d{idx}.length, ...
                results_3d{idx}.nodes, results_3d{idx}.smoothness);
    else
        fprintf('%-16s  %10s  %12s  %8s  %12s\n', algorithms{idx}.name, '失败', '-', '-', '-');
    end
end
fprintf('================================================================\n\n');

%% =================== 保存图片 ===================
save_dir = 'results';
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
saveas(fig_2d,  fullfile(save_dir, sprintf('comparison_2D_%s.png', timestamp)));
saveas(fig_3d,  fullfile(save_dir, sprintf('comparison_3D_%s.png', timestamp)));
saveas(fig_bar, fullfile(save_dir, sprintf('performance_bars_%s.png', timestamp)));

% 高分辨率版本
print(fig_2d,  fullfile(save_dir, sprintf('comparison_2D_%s_hires.png', timestamp)), '-dpng', '-r300');
print(fig_3d,  fullfile(save_dir, sprintf('comparison_3D_%s_hires.png', timestamp)), '-dpng', '-r300');
print(fig_bar, fullfile(save_dir, sprintf('performance_bars_%s_hires.png', timestamp)), '-dpng', '-r300');

fprintf('图片已保存至 %s/ 目录\n', save_dir);

% 保存数据
save(fullfile(save_dir, sprintf('experiment_data_%s.mat', timestamp)), ...
     'results_2d', 'results_3d', 'env_2d', 'env_3d', 'config', 'algorithms');
fprintf('数据已保存至 %s/experiment_data_%s.mat\n', save_dir, timestamp);

fprintf('\n================================================================\n');
fprintf('                  实验完成!\n');
fprintf('================================================================\n\n');
fprintf('总共打开了3个窗口:\n');
fprintf('  窗口1: 2D 路径规划结果对比 (%d个算法subplot)\n', num_alg);
fprintf('  窗口2: 3D 路径规划结果对比 (%d个算法subplot)\n', num_alg);
fprintf('  窗口3: 性能指标柱状图对比\n');
fprintf('\n');


%% =================== 算法封装函数 ===================

function [path, tree, success, metrics] = run_rrt(env, max_iter)
    tic;
    [path, tree, success] = RRT_Basic(env, max_iter);
    t = toc;
    metrics = struct('planning_time', t, 'path_length', inf, 'tree_nodes', size(tree.vertices,1), ...
                     'iterations', max_iter, 'smoothness', NaN, 'clearance', NaN);
    if success && ~isempty(path)
        metrics.path_length = tree.final_cost;
    end
end

function [path, tree, success, metrics] = run_rrt_connect(env, max_iter)
    tic;
    [path, tree, success] = RRT_Connect_Basic(env, max_iter);
    t = toc;
    metrics = struct('planning_time', t, 'path_length', inf, 'tree_nodes', size(tree.vertices,1), ...
                     'iterations', max_iter, 'smoothness', NaN, 'clearance', NaN);
    if success && ~isempty(path)
        metrics.path_length = tree.final_cost;
    end
end

function [path, tree, success, metrics] = run_rrt_star(env, max_iter)
    tic;
    [path, tree, success] = RRT_Star_Basic(env, max_iter);
    t = toc;
    metrics = struct('planning_time', t, 'path_length', inf, 'tree_nodes', size(tree.vertices,1), ...
                     'iterations', max_iter, 'smoothness', NaN, 'clearance', NaN);
    if success && ~isempty(path)
        metrics.path_length = tree.final_cost;
    end
end

function [path, tree, success, metrics] = run_sc_rrt(env, max_iter)
    [path, tree, success, metrics] = SC_RRT_Basic(env, max_iter);
end

function [path, tree, success, metrics] = run_dynamic_rrt(env, max_iter)
    [path, tree, success, metrics] = Dynamic_RRT_Basic(env, max_iter);
end

function smoothness = calcSmoothness(path)
    if isempty(path) || size(path,1) < 3
        smoothness = NaN;
        return;
    end
    angles = zeros(size(path,1)-2, 1);
    for i = 2:size(path,1)-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        cosA = dot(v1,v2) / (norm(v1)*norm(v2) + eps);
        cosA = max(-1, min(1, cosA));
        angles(i-1) = acos(cosA);
    end
    smoothness = var(angles);
end

function nodeCount = getNodeCount(tree)
    if isfield(tree, 'vertices')
        nodeCount = size(tree.vertices, 1);
    elseif isfield(tree, 'nodes')
        nodeCount = size(tree.nodes, 1);
    else
        nodeCount = 0;
    end
end
