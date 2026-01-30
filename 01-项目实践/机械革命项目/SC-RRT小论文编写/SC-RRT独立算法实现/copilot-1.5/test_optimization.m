% =========================================================================
%          快速验证脚本 - 节点重连和帕累托优化效果测试
% =========================================================================
% 功能: 快速运行一次算法，观察三个关键改进
% 1. 节点重连优化效果（终端输出）
% 2. 帕累托最优节点可视化（橙色标记）
% 3. 最终路径质量对比
% =========================================================================
clear; clc; close all;

fprintf('\n========================================\n');
fprintf('  SC-RRT 优化功能验证测试\n');
fprintf('========================================\n\n');

%% 1. 环境配置（简单2D环境，快速验证）
dimension = '2D';
bounds = [0 1000 0 1000];
startPoint = [200 200];
goalPoint = [800 800];

% 生成简单障碍物环境
rng(42);  % 固定随机种子，便于对比
obstacles = generateObstacles(dimension, bounds, 150, 12, 42);

%% 2. 可视化设置
fig = figure('Name', 'SC-RRT优化验证', 'Position', [100 100 1000 800]);
hold on; grid on; axis equal;
title('SC-RRT优化功能验证 (橙色=帕累托最优节点)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');

% 绘制障碍物
for i = 1:size(obstacles.positions, 1)
    pos = obstacles.positions(i, :);
    rad = obstacles.radii(i);
    rectangle('Position', [pos(1)-rad, pos(2)-rad, 2*rad, 2*rad], ...
        'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');
end

% 绘制起点和终点
scatter(startPoint(1), startPoint(2), 200, 'g', 'filled', 'pentagram', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Start');
scatter(goalPoint(1), goalPoint(2), 200, 'r', 'filled', 'pentagram', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Goal');

legend('Location', 'best');
drawnow;

%% 3. 运行SC-RRT算法（启用所有优化功能）
fprintf('========== 开始规划 ==========\n');
fprintf('预期观察：\n');
fprintf('1. 橙色圆点标记（每100次迭代更新）= 帕累托最优节点\n');
fprintf('2. 终端输出"路径重连优化"信息\n');
fprintf('3. 最终路径更加平滑且节点更少\n\n');

tic;
[treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
    startPoint, goalPoint, bounds, obstacles, fig, '', 10, 0.05, ...
    'Mode', 'adaptive', ...               % 自适应模式
    'MaxIterations', 3000, ...            % 最大迭代次数
    'UpdateInterval', 50, ...             % 交汇点更新间隔
    'UseParetoFrontier', true, ...        % 启用帕累托前沿
    'VisualizeDualEllipsoid', true, ...   % 可视化双椭球
    'EnableVisualization', true, ...      % 启用实时可视化
    'VisualizationInterval', 5 ...        % 可视化间隔（降低以加快速度）
);
planningTime = toc;

%% 4. 结果分析和可视化
fprintf('\n========== 算法执行结果 ==========\n');
if success
    fprintf('✅ 规划成功！\n');
    fprintf('总迭代次数: %d\n', metrics.iterCount);
    fprintf('计算时间: %.3f 秒\n', planningTime);
    fprintf('树A节点数: %d\n', metrics.treeASize);
    fprintf('树B节点数: %d\n', metrics.treeBSize);
    fprintf('总节点数: %d\n', metrics.totalNodes);
    fprintf('\n--- 路径质量指标 ---\n');
    fprintf('路径长度: %.2f\n', metrics.pathLength);
    fprintf('路径节点数: %d\n', metrics.pathNodes);
    fprintf('路径平滑度: %.4f (越小越平滑)\n', metrics.pathSmoothness);
    fprintf('有效采样率: %.2f%%\n', metrics.validSamplingRate * 100);
    
    % 绘制最终路径（突出显示）
    figure(fig);
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 4, 'DisplayName', '最终路径');
    
    % 标记路径关键节点
    plot(path(:,1), path(:,2), 'mo', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'm', 'DisplayName', '路径节点');
    
    legend('Location', 'best');
    title(sprintf('SC-RRT优化验证 (路径长度=%.1f, 节点数=%d)', ...
        metrics.pathLength, metrics.pathNodes), ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    fprintf('\n========================================\n');
    fprintf('  验证要点总结\n');
    fprintf('========================================\n');
    fprintf('1. 橙色圆点: 规划过程中显示的帕累托最优节点\n');
    fprintf('2. 紫色圆圈: 最终路径的所有节点（应该较少）\n');
    fprintf('3. 红色粗线: 优化后的最终路径（应该较平滑）\n');
    fprintf('4. 查看上方是否有"路径重连优化"的输出信息\n');
    fprintf('========================================\n\n');
    
else
    fprintf('❌ 规划失败\n');
    fprintf('总迭代次数: %d\n', metrics.iterCount);
    fprintf('计算时间: %.3f 秒\n', planningTime);
    fprintf('建议: 增加最大迭代次数或降低障碍物密度\n');
end

%% 5. 性能对比提示
fprintf('\n========== 性能对比建议 ==========\n');
fprintf('要对比优化效果，建议：\n');
fprintf('1. 记录当前的路径节点数: %d\n', metrics.pathNodes);
fprintf('2. 注释掉rewirePathNodes调用（SC_RRT_Bidirectional.m中）\n');
fprintf('3. 重新运行本脚本\n');
fprintf('4. 对比优化前后的节点数差异\n');
fprintf('预期: 节点数减少20%%-50%%\n');
fprintf('==================================\n\n');
