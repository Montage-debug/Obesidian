%% 可视化诊断 - 检查SC-RRT路径连续性
clear; clc; close all;

% 加载最新的SC-RRT结果
files = dir('E:\Matlab练习\RRT对比实验 -1. 4 初步实现四个算法同一环境下实现 copy\rrt_toolbox-master\results\SC_RRT*2D*.mat');
if isempty(files)
    error('未找到SC-RRT结果文件');
end
[~, idx] = max([files.datenum]);
latest_file = fullfile(files(idx).folder, files(idx).name);
fprintf('加载文件: %s\n', files(idx).name);
load(latest_file);

fprintf('\n=== 路径连续性检查 ===\n');
fprintf('路径节点数: %d\n', size(path, 1));
fprintf('树节点数: %d\n', size(tree.vertices, 1));

% 检查路径连续性
fprintf('\n路径段距离:\n');
for i = 1:min(10, size(path,1)-1)
    dist = norm(path(i+1,:) - path(i,:));
    fprintf('  %d → %d: %.4f\n', i, i+1, dist);
end

% 检查树结构
fprintf('\n树parent关系（前10）:\n');
for i = 1:min(10, size(tree.parent,1))
    fprintf('  节点%d: parent=%d\n', i, tree.parent(i));
end

% 可视化对比
figure('Position', [100 100 1400 600]);

% 子图1：原始绘制方式
subplot(1,2,1);
hold on; axis equal; grid on;
title('原始绘制（可能有交叉）');

% 绘制树边
for i = 2:size(tree.vertices, 1)
    parent_idx = tree.parent(i);
    if parent_idx > 0
        plot([tree.vertices(parent_idx, 1), tree.vertices(i, 1)], ...
             [tree.vertices(parent_idx, 2), tree.vertices(i, 2)], ...
             'b-', 'LineWidth', 1);
    end
end

% 绘制路径
plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 3);
plot(path(1, 1), path(1, 2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(path(end, 1), path(end, 2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

% 子图2：仅路径
subplot(1,2,2);
hold on; axis equal; grid on;
title('仅显示最终路径');

% 只绘制路径
plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 3);
plot(path(:, 1), path(:, 2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
plot(path(1, 1), path(1, 2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(path(end, 1), path(end, 2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

% 标注节点编号（每5个）
for i = 1:5:size(path,1)
    text(path(i,1), path(i,2), sprintf('%d', i), ...
         'FontSize', 8, 'Color', 'blue', 'FontWeight', 'bold');
end

fprintf('\n检查完成！请查看图形窗口\n');
