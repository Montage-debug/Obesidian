% =========================================================================
%     五算法对比实验 - 一键运行脚本
% =========================================================================
% 功能: 一键对比5种RRT算法变体的性能
%
% 支持算法:
%   1. RRT_Basic          - 原生RRT算法
%   2. RRT_Connect_Basic  - RRT-Connect算法 (双向快速连接)
%   3. RRT_Star_Basic     - RRT*算法 (渐进最优)
%   4. SC_RRT_Basic       - SC-RRT算法 (双向+PID+Pareto)
%   5. Dynamic_RRT_Basic  - Dynamic-RRT算法 (Informed+Pareto动态规划)
%
% 输出:
%   - 每个算法的独立结果图 (标题含 时间/路径长度/节点数/平滑度)
%   - 综合对比图 (所有算法路径叠加显示)
%   - 性能柱状图对比
%   - 控制台输出完整统计表
%
% 使用方法:
%   直接在MATLAB中运行本脚本即可
% =========================================================================
clear; clc; close all;

%% =================== 环境配置 ===================
fprintf('\n');
fprintf('================================================================\n');
fprintf('           五算法对比实验 - 一键运行\n');
fprintf('================================================================\n\n');

% ===== 用户可修改区域 =====
config.dimension   = '2D';             % '2D' 或 '3D'
config.bounds_2d   = [0 1500 0 1500];  % 2D空间边界 [xmin xmax ymin ymax]
config.bounds_3d   = [0 1500 0 1500 0 1500]; % 3D空间边界
config.start_2d    = [400 400];        % 2D起点
config.goal_2d     = [1100 1100];      % 2D终点
config.start_3d    = [200 200 200];    % 3D起点
config.goal_3d     = [1300 1300 1300]; % 3D终点
config.num_obs     = 225;              % 障碍物数量
config.obs_radius  = 15;              % 障碍物基准半径
config.max_iter    = 5000;            % 最大迭代次数
config.seed        = 42;              % 随机种子 (设为'shuffle'则每次随机)
% ===========================

is3D = strcmp(config.dimension, '3D');

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

%% =================== 生成环境 ===================
fprintf('\n--- 生成障碍物环境 ---\n');

if ischar(config.seed) && strcmp(config.seed, 'shuffle')
    rng('shuffle');
else
    rng(config.seed);
end

if is3D
    bounds = config.bounds_3d;
    startPoint = config.start_3d;
    goalPoint  = config.goal_3d;
else
    bounds = config.bounds_2d;
    startPoint = config.start_2d;
    goalPoint  = config.goal_2d;
end

% 使用generate2DEnvironment/generate3DEnvironment生成环境
opts = struct();
opts.start_point = startPoint;
opts.goal_point  = goalPoint;
opts.fixed_radius = config.obs_radius;
opts.min_spacing = 5.0;
opts.clearance   = 45.0;

if is3D
    env = EnvironmentConfig.generate3DEnvironment(bounds, config.num_obs, opts);
else
    env = EnvironmentConfig.generate2DEnvironment(bounds, config.num_obs, opts);
end

fprintf('环境生成完成: %d个障碍物\n', env.num);

%% =================== 显示环境图 ===================
fig_env = figure('Name', '障碍物环境', 'Position', [50 100 800 700], 'Color', 'w');
hold on; axis equal; grid on;
xlim(bounds(1:2)); ylim(bounds(3:4));
xlabel('X (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y (mm)', 'FontSize', 12, 'FontWeight', 'bold');

if is3D
    zlim(bounds(5:6));
    zlabel('Z (mm)', 'FontSize', 12, 'FontWeight', 'bold');
    view(3);
    for i = 1:env.num
        [Xs,Ys,Zs] = sphere(12);
        surf(Xs*env.obstacles(i,4)+env.obstacles(i,1), ...
             Ys*env.obstacles(i,4)+env.obstacles(i,2), ...
             Zs*env.obstacles(i,4)+env.obstacles(i,3), ...
             'FaceColor', [0 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    end
    plot3(startPoint(1), startPoint(2), startPoint(3), 'go', ...
          'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot3(goalPoint(1), goalPoint(2), goalPoint(3), 'rs', ...
          'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
else
    for i = 1:env.num
        obs = env.obstacles(i, :);
        rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
                 'Curvature', [1 1], 'FaceColor', [0 0 0], 'EdgeColor', [0 0 0], 'LineWidth', 0.5);
    end
    plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(goalPoint(1), goalPoint(2), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
end
title(sprintf('障碍物环境 (%s, %d个障碍物)', config.dimension, env.num), ...
      'FontSize', 14, 'FontWeight', 'bold');
drawnow;

%% =================== 定义算法 ===================
algorithms = {
    struct('name', 'RRT',          'func', @(e,n) run_rrt(e,n),          'color', [0.85 0.33 0.10]);
    struct('name', 'RRT-Connect',  'func', @(e,n) run_rrt_connect(e,n),  'color', [0.93 0.69 0.13]);
    struct('name', 'RRT*',         'func', @(e,n) run_rrt_star(e,n),     'color', [0.47 0.67 0.19]);
    struct('name', 'SC-RRT',       'func', @(e,n) run_sc_rrt(e,n),       'color', [0.00 0.45 0.74]);
    struct('name', 'Dynamic-RRT',  'func', @(e,n) run_dynamic_rrt(e,n),  'color', [1.00 0.00 0.00]);
};

num_alg = length(algorithms);

%% =================== 运行所有算法 ===================
fprintf('\n================================================================\n');
fprintf('开始运行 %d 个算法...\n', num_alg);
fprintf('================================================================\n\n');

results = cell(num_alg, 1);

for idx = 1:num_alg
    alg = algorithms{idx};
    fprintf('【%d/%d】 %s 运行中...\n', idx, num_alg, alg.name);
    
    % 设置独立随机种子 (保证每次运行一致)
    if isnumeric(config.seed)
        rng(config.seed + 100 + idx);
    end
    
    try
        tic;
        [path, tree, success, metrics] = alg.func(env, config.max_iter);
        elapsed = toc;
        
        if success && ~isempty(path)
            % 计算路径长度
            pathLen = 0;
            for i = 1:size(path,1)-1
                pathLen = pathLen + norm(path(i+1,:) - path(i,:));
            end
            
            % 计算平滑度
            smooth_val = calcSmoothness(path);
            
            % 节点数
            if isfield(tree, 'vertices')
                nodeCount = size(tree.vertices, 1);
            elseif isfield(tree, 'nodes')
                nodeCount = size(tree.nodes, 1);
            else
                nodeCount = 0;
            end
            
            results{idx} = struct('success', true, 'path', path, 'tree', tree, ...
                                  'time', elapsed, 'length', pathLen, ...
                                  'nodes', nodeCount, 'smoothness', smooth_val);
            
            fprintf('  >> 成功! Time:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f\n', ...
                    elapsed, pathLen, nodeCount, smooth_val);
        else
            results{idx} = struct('success', false);
            fprintf('  >> 失败 (未找到路径)\n');
        end
    catch ME
        results{idx} = struct('success', false);
        fprintf('  >> 错误: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('     位置: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
        end
    end
    fprintf('\n');
end

%% =================== 综合对比图 ===================
fprintf('================================================================\n');
fprintf('生成对比结果图...\n');
fprintf('================================================================\n\n');

% 计算子图布局
if num_alg <= 3
    nrows = 1; ncols = num_alg;
elseif num_alg <= 6
    nrows = 2; ncols = ceil(num_alg / 2);
else
    nrows = 3; ncols = ceil(num_alg / 3);
end

fig_compare = figure('Name', '算法对比结果', ...
                     'Position', [50 50 ncols*450 nrows*400], 'Color', 'w');

for idx = 1:num_alg
    alg = algorithms{idx};
    res = results{idx};
    
    subplot(nrows, ncols, idx);
    hold on; axis equal; grid on;
    xlim(bounds(1:2)); ylim(bounds(3:4));
    xlabel('X (mm)', 'FontSize', 11); ylabel('Y (mm)', 'FontSize', 11);
    
    if is3D
        zlim(bounds(5:6)); zlabel('Z (mm)', 'FontSize', 11); view(3);
    end
    
    % 绘制障碍物
    if is3D
        for i = 1:env.num
            [Xs,Ys,Zs] = sphere(8);
            surf(Xs*env.obstacles(i,4)+env.obstacles(i,1), ...
                 Ys*env.obstacles(i,4)+env.obstacles(i,2), ...
                 Zs*env.obstacles(i,4)+env.obstacles(i,3), ...
                 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        end
    else
        for i = 1:env.num
            obs = env.obstacles(i, :);
            rectangle('Position', [obs(1)-obs(3), obs(2)-obs(3), 2*obs(3), 2*obs(3)], ...
                     'Curvature', [1 1], 'FaceColor', [0.15 0.15 0.15], ...
                     'EdgeColor', 'none');
        end
    end
    
    % 绘制起终点
    if is3D
        plot3(startPoint(1), startPoint(2), startPoint(3), 'go', ...
              'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
        plot3(goalPoint(1), goalPoint(2), goalPoint(3), 'rs', ...
              'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    else
        plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
        plot(goalPoint(1), goalPoint(2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    end
    
    % 绘制路径 + 标题
    if res.success
        path = res.path;
        if is3D && size(path,2) >= 3
            plot3(path(:,1), path(:,2), path(:,3), 'Color', alg.color, 'LineWidth', 2.5);
        else
            plot(path(:,1), path(:,2), 'Color', alg.color, 'LineWidth', 2.5);
        end
        title_str = sprintf('%s\nTime:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f', ...
                           alg.name, res.time, res.length, res.nodes, res.smoothness);
        title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    else
        title(sprintf('%s\n(规划失败)', alg.name), 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
    end
    
    set(gca, 'FontSize', 10);
end

drawnow;

%% =================== 性能柱状图 ===================
fig_bar = figure('Name', '性能指标对比', 'Position', [100 100 1200 500], 'Color', 'w');

% 收集数据
alg_names = {};
times = []; lengths = []; nodes = []; smooths = [];

for idx = 1:num_alg
    if results{idx}.success
        alg_names{end+1} = algorithms{idx}.name; %#ok<SAGROW>
        times(end+1)   = results{idx}.time; %#ok<SAGROW>
        lengths(end+1) = results{idx}.length; %#ok<SAGROW>
        nodes(end+1)   = results{idx}.nodes; %#ok<SAGROW>
        smooths(end+1) = results{idx}.smoothness; %#ok<SAGROW>
    end
end

if ~isempty(alg_names)
    colors_bar = [];
    for idx = 1:num_alg
        if results{idx}.success
            colors_bar = [colors_bar; algorithms{idx}.color]; %#ok<AGROW>
        end
    end
    
    subplot(1, 4, 1);
    b1 = bar(times);
    b1.FaceColor = 'flat';
    for k = 1:length(times), b1.CData(k,:) = colors_bar(k,:); end
    set(gca, 'XTickLabel', alg_names, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Time (s)'); title('计算时间', 'FontSize', 12, 'FontWeight', 'bold');
    
    subplot(1, 4, 2);
    b2 = bar(lengths);
    b2.FaceColor = 'flat';
    for k = 1:length(lengths), b2.CData(k,:) = colors_bar(k,:); end
    set(gca, 'XTickLabel', alg_names, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Length (mm)'); title('路径长度', 'FontSize', 12, 'FontWeight', 'bold');
    
    subplot(1, 4, 3);
    b3 = bar(nodes);
    b3.FaceColor = 'flat';
    for k = 1:length(nodes), b3.CData(k,:) = colors_bar(k,:); end
    set(gca, 'XTickLabel', alg_names, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Nodes'); title('搜索节点数', 'FontSize', 12, 'FontWeight', 'bold');
    
    subplot(1, 4, 4);
    b4 = bar(smooths);
    b4.FaceColor = 'flat';
    for k = 1:length(smooths), b4.CData(k,:) = colors_bar(k,:); end
    set(gca, 'XTickLabel', alg_names, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Smoothness'); title('平滑度', 'FontSize', 12, 'FontWeight', 'bold');
    
    sgtitle('性能指标对比', 'FontSize', 14, 'FontWeight', 'bold');
end

drawnow;

%% =================== 控制台统计表 ===================
fprintf('\n================================================================\n');
fprintf('                    性能统计表\n');
fprintf('================================================================\n');
fprintf('%-16s  %10s  %12s  %8s  %12s\n', '算法', '时间(s)', '路径长度(mm)', '节点数', '平滑度');
fprintf('----------------------------------------------------------------\n');

for idx = 1:num_alg
    if results{idx}.success
        fprintf('%-16s  %10.4f  %12.2f  %8d  %12.4f\n', ...
                algorithms{idx}.name, results{idx}.time, results{idx}.length, ...
                results{idx}.nodes, results{idx}.smoothness);
    else
        fprintf('%-16s  %10s  %12s  %8s  %12s\n', algorithms{idx}.name, '失败', '-', '-', '-');
    end
end
fprintf('================================================================\n\n');

%% =================== 保存图片 ===================
save_dir = 'results';
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
saveas(fig_env,     fullfile(save_dir, sprintf('environment_%s.png', timestamp)));
saveas(fig_compare, fullfile(save_dir, sprintf('comparison_%s.png', timestamp)));
saveas(fig_bar,     fullfile(save_dir, sprintf('performance_%s.png', timestamp)));

fprintf('图片已保存至 %s/ 目录\n', save_dir);
fprintf('================================================================\n');
fprintf('                  实验完成!\n');
fprintf('================================================================\n\n');


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
    angles = [];
    for i = 2:size(path,1)-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        cosA = dot(v1,v2) / (norm(v1)*norm(v2) + eps);
        cosA = max(-1, min(1, cosA));
        angles(end+1) = acos(cosA); %#ok<AGROW>
    end
    smoothness = var(angles);
end
