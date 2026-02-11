% =========================================================================
%     算法对比演示脚本 - 单次可视化对比
% =========================================================================
% 用途: 可视化展示不同算法在同一环境下的规划结果
%       适合演示和理解算法差异
% =========================================================================
clear; clc; close all;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║          算法对比可视化演示                                 ║\n');
fprintf('║      Single Run Visualization Comparison                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 环境配置（与论文一致）
config = struct();
config.dimension = '2D';
config.bounds = [0 1500 0 1500];
config.startPoint = [400 400];
config.goalPoint = [1100 1100];
config.numObstacles = 225;
config.obstacleRadius = 15;
config.seed = 42;  % 固定种子确保可复现

fprintf('环境配置:\n');
fprintf('  空间: [%d %d] × [%d %d]\n', config.bounds);
fprintf('  起点: (%.0f, %.0f)\n', config.startPoint);
fprintf('  终点: (%.0f, %.0f)\n', config.goalPoint);
fprintf('  障碍物: %d个 (半径%d)\n\n', config.numObstacles, config.obstacleRadius);

%% 生成障碍物环境
fprintf('生成障碍物环境...\n');
rng(config.seed);

% 尝试使用SC-RRT的generateObstacles
sc_rrt_path = '../SC-RRT独立算法实现/copilot-1.6';
if exist(fullfile(sc_rrt_path, 'generateObstacles.m'), 'file')
    addpath(sc_rrt_path);
    obstacles = generateObstacles(config.dimension, config.bounds, ...
                                 config.numObstacles, config.obstacleRadius, ...
                                 config.startPoint, config.goalPoint);
else
    obstacles = GenerateObstacles(config.dimension, config.bounds, ...
                                 config.numObstacles, config.obstacleRadius, ...
                                 config.startPoint, config.goalPoint, config.seed);
end

fprintf('障碍物生成完成\n\n');

%% 定义要演示的算法
demo_algorithms = {
    struct('name', 'Dynamic-RRT', 'type', 'drrt', 'interval', 4, 'subplot_idx', 1),
    struct('name', 'SC-RRT Basic', 'type', 'sc', 'mode', 'basic', 'subplot_idx', 2),
    struct('name', 'SC-RRT with PID', 'type', 'sc', 'mode', 'pid', 'subplot_idx', 3),
    struct('name', 'SC-RRT Adaptive', 'type', 'sc', 'mode', 'adaptive', 'subplot_idx', 4)
};

num_alg = length(demo_algorithms);
results = cell(num_alg, 1);

%% 运行算法并记录结果
fprintf('运行算法对比...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

for i = 1:num_alg
    alg = demo_algorithms{i};
    fprintf('%d. 运行 %s ... ', i, alg.name);
    
    try
        if strcmp(alg.type, 'drrt')
            % Dynamic RRT
            [tree, path, success, metrics] = DynamicRRT(...
                config.startPoint, config.goalPoint, config.bounds, obstacles, ...
                'Interval', alg.interval, ...
                'EnableVisualization', false);
            
            if success
                results{i}.success = true;
                results{i}.tree = tree;
                results{i}.path = path;
                results{i}.time = metrics.convergenceTime;
                results{i}.length = metrics.pathLength;
                results{i}.nodes = metrics.nodeCount;
                fprintf('成功 (%.4fs, 长度%.2f, %d节点)\n', ...
                        metrics.convergenceTime, metrics.pathLength, metrics.nodeCount);
            else
                results{i}.success = false;
                fprintf('失败\n');
            end
            
        elseif strcmp(alg.type, 'sc')
            % SC-RRT
            fig_temp = figure('Visible', 'off');
            [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
                config.startPoint, config.goalPoint, config.bounds, obstacles, ...
                fig_temp, '', 0, 0, ...
                'Mode', alg.mode, ...
                'EnableVisualization', false);
            close(fig_temp);
            
            if success
                results{i}.success = true;
                results{i}.treeA = treeA;
                results{i}.treeB = treeB;
                results{i}.path = path;
                results{i}.time = metrics.planningTime;
                results{i}.length = metrics.finalPathLength;
                results{i}.nodes = treeA.count + treeB.count;
                fprintf('成功 (%.4fs, 长度%.2f, %d节点)\n', ...
                        metrics.planningTime, metrics.finalPathLength, ...
                        treeA.count + treeB.count);
            else
                results{i}.success = false;
                fprintf('失败\n');
            end
        end
        
    catch ME
        results{i}.success = false;
        fprintf('错误: %s\n', ME.message);
    end
end

fprintf('\n');

%% 可视化对比
fprintf('生成可视化对比图...\n');

% 创建大图
fig = figure('Name', '算法对比可视化', 'Position', [50 50 1400 1000], 'Color', 'w');

% 颜色配置
colors = [
    0.85 0.33 0.10;  % Dynamic RRT - 橙红
    0.47 0.67 0.19;  % SC Basic - 绿
    0.30 0.75 0.93;  % SC PID - 天蓝
    0.00 0.45 0.74;  % SC Adaptive - 深蓝
];

for i = 1:num_alg
    if ~results{i}.success
        continue;
    end
    
    alg = demo_algorithms{i};
    subplot(2, 2, alg.subplot_idx);
    hold on; grid on; axis equal;
    xlim([config.bounds(1) config.bounds(2)]);
    ylim([config.bounds(3) config.bounds(4)]);
    
    % 绘制障碍物
    for j = 1:size(obstacles.circles, 1)
        rectangle('Position', [obstacles.circles(j,1:2)-obstacles.circles(j,3), ...
                 2*obstacles.circles(j,3), 2*obstacles.circles(j,3)], ...
                 'Curvature', [1 1], 'FaceColor', [0.85 0.85 0.85], ...
                 'EdgeColor', 'none');
    end
    
    % 绘制搜索树
    if strcmp(alg.type, 'drrt')
        % Dynamic RRT: 单树
        tree = results{i}.tree;
        for k = 2:tree.count
            parent = tree.parents(k);
            plot([tree.nodes(parent,1) tree.nodes(k,1)], ...
                 [tree.nodes(parent,2) tree.nodes(k,2)], ...
                 '-', 'Color', [colors(i,:) 0.2], 'LineWidth', 0.3);
        end
    else
        % SC-RRT: 双树
        treeA = results{i}.treeA;
        treeB = results{i}.treeB;
        % 树A（从起点）- 蓝色系
        for k = 2:treeA.count
            parent = treeA.parent(k);
            plot([treeA.nodes(parent,1) treeA.nodes(k,1)], ...
                 [treeA.nodes(parent,2) treeA.nodes(k,2)], ...
                 '-', 'Color', [[0.2 0.4 0.8] 0.2], 'LineWidth', 0.3);
        end
        % 树B（从终点）- 红色系
        for k = 2:treeB.count
            parent = treeB.parent(k);
            plot([treeB.nodes(parent,1) treeB.nodes(k,1)], ...
                 [treeB.nodes(parent,2) treeB.nodes(k,2)], ...
                 '-', 'Color', [[0.8 0.2 0.2] 0.2], 'LineWidth', 0.3);
        end
    end
    
    % 绘制最终路径
    path = results{i}.path;
    plot(path(:,1), path(:,2), '-', 'Color', colors(i,:), 'LineWidth', 3);
    
    % 绘制起点和终点
    plot(config.startPoint(1), config.startPoint(2), 'o', ...
         'MarkerSize', 12, 'MarkerFaceColor', [0 0.7 0], ...
         'MarkerEdgeColor', 'k', 'LineWidth', 2);
    plot(config.goalPoint(1), config.goalPoint(2), 'o', ...
         'MarkerSize', 12, 'MarkerFaceColor', [0.8 0 0], ...
         'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    % 标题和标注
    title(sprintf('%s\nTime: %.4fs | Length: %.2f | Nodes: %d', ...
                  alg.name, results{i}.time, results{i}.length, results{i}.nodes), ...
          'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X (mm)');
    ylabel('Y (mm)');
    set(gca, 'FontSize', 10);
end

% 添加总标题
sgtitle('Path Planning Algorithm Comparison', 'FontSize', 14, 'FontWeight', 'bold');

% 保存图像
saveas(fig, 'algorithm_comparison_demo.png');
saveas(fig, 'algorithm_comparison_demo.fig');

fprintf('可视化对比图已保存: algorithm_comparison_demo.png\n\n');

%% 生成对比表格
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('                        对比结果\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

fprintf('%-25s | %-12s | %-12s | %-12s\n', '算法', '收敛时间(s)', '路径长度', '节点数');
fprintf('%s\n', repmat('-', 70, 1));

for i = 1:num_alg
    if results{i}.success
        fprintf('%-25s | %12.4f | %12.2f | %12d\n', ...
                demo_algorithms{i}.name, ...
                results{i}.time, ...
                results{i}.length, ...
                results{i}.nodes);
    else
        fprintf('%-25s | %12s | %12s | %12s\n', ...
                demo_algorithms{i}.name, '失败', '失败', '失败');
    end
end

fprintf('\n');

%% 生成性能对比柱状图
fprintf('生成性能对比柱状图...\n');

fig2 = figure('Position', [100 100 1000 400], 'Color', 'w');

% 提取数据
times = zeros(num_alg, 1);
lengths = zeros(num_alg, 1);
nodes = zeros(num_alg, 1);
names = cell(num_alg, 1);

for i = 1:num_alg
    names{i} = demo_algorithms{i}.name;
    if results{i}.success
        times(i) = results{i}.time;
        lengths(i) = results{i}.length;
        nodes(i) = results{i}.nodes;
    else
        times(i) = NaN;
        lengths(i) = NaN;
        nodes(i) = NaN;
    end
end

% 时间对比
subplot(1, 3, 1);
bar(times, 'FaceColor', 'flat', 'CData', colors, 'EdgeColor', 'k');
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 30);
ylabel('收敛时间 (秒)');
title('(a) Convergence Time', 'FontWeight', 'bold');
grid on;

% 长度对比
subplot(1, 3, 2);
bar(lengths, 'FaceColor', 'flat', 'CData', colors, 'EdgeColor', 'k');
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 30);
ylabel('路径长度');
title('(b) Path Length', 'FontWeight', 'bold');
grid on;

% 节点数对比
subplot(1, 3, 3);
bar(nodes, 'FaceColor', 'flat', 'CData', colors, 'EdgeColor', 'k');
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 30);
ylabel('节点数');
title('(c) Node Count', 'FontWeight', 'bold');
grid on;

saveas(fig2, 'performance_comparison_bar.png');

fprintf('性能对比柱状图已保存: performance_comparison_bar.png\n\n');

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('演示完成! 🎉\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n提示: 运行 run_comparison_experiment 进行完整的统计实验\n\n');
