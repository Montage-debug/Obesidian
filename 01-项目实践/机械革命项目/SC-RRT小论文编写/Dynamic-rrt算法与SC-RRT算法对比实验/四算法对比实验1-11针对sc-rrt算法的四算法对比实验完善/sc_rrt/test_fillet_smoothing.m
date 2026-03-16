%% test_fillet_smoothing.m - 测试圆角平滑功能
% 
% 功能：验证圆角平滑处理在不同场景下的表现
% 测试内容：
%   1. 2D环境下的圆角平滑
%   2. 3D环境下的圆角平滑  
%   3. 障碍物碰撞检测
%   4. 不同圆角半径的效果
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

clear; close all; clc;
addpath(genpath('./sc_rrt'));

%% 测试1: 2D简单路径的圆角处理
fprintf('========== 测试1: 2D简单路径圆角处理 ==========\n');

% 创建一个带拐点的简单路径
path_2d = [
    0, 0;
    10, 0;
    10, 10;
    20, 10;
    20, 20
];

% 无障碍物
obstacles_2d = [];
dim_2d = 2;

% 测试不同的圆角半径
fillet_radii = [1, 2, 3];

figure('Name', '2D路径圆角处理', 'Position', [100, 100, 1200, 400]);

for i = 1:length(fillet_radii)
    subplot(1, 3, i);
    
    % 执行圆角处理
    [path_smooth, success] = smoothPathWithFillets(path_2d, obstacles_2d, dim_2d, fillet_radii(i), 15);
    
    % 绘制原路径
    plot(path_2d(:,1), path_2d(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '原路径');
    hold on;
    
    % 绘制圆角路径
    plot(path_smooth(:,1), path_smooth(:,2), 'r-', 'LineWidth', 2, 'DisplayName', '圆角路径');
    
    % 标记起点和终点
    plot(path_2d(1,1), path_2d(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    plot(path_2d(end,1), path_2d(end,2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', '终点');
    
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title(sprintf('圆角半径 = %.1f', fillet_radii(i)));
    legend('Location', 'best');
    
    % 输出信息
    fprintf('  圆角半径 %.1f: 节点数 %d -> %d, 成功: %d\n', ...
        fillet_radii(i), size(path_2d, 1), size(path_smooth, 1), success);
end

%% 测试2: 2D带障碍物的圆角处理
fprintf('\n========== 测试2: 2D带障碍物的圆角处理 ==========\n');

% 创建路径
path_2d_obs = [
    0, 0;
    10, 5;
    10, 15;
    20, 10;
    30, 15;
    40, 10
];

% 添加障碍物（靠近拐点）
obstacles_2d_obs = [
    12, 10, 2.5;   % [x, y, radius]
    18, 12, 2;
    28, 12, 2
];

figure('Name', '2D带障碍物的圆角处理', 'Position', [100, 600, 800, 600]);

% 执行圆角处理
fillet_radius = 2.0;
[path_smooth_obs, success_obs] = smoothPathWithFillets(path_2d_obs, obstacles_2d_obs, dim_2d, fillet_radius, 15);

% 绘制障碍物
for i = 1:size(obstacles_2d_obs, 1)
    theta = linspace(0, 2*pi, 50);
    x_circle = obstacles_2d_obs(i, 1) + obstacles_2d_obs(i, 3) * cos(theta);
    y_circle = obstacles_2d_obs(i, 2) + obstacles_2d_obs(i, 3) * sin(theta);
    fill(x_circle, y_circle, [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'DisplayName', '障碍物');
    hold on;
end

% 绘制原路径
plot(path_2d_obs(:,1), path_2d_obs(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '原路径');

% 绘制圆角路径
plot(path_smooth_obs(:,1), path_smooth_obs(:,2), 'r-', 'LineWidth', 2, 'DisplayName', '圆角路径');

% 标记起点和终点
plot(path_2d_obs(1,1), path_2d_obs(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(path_2d_obs(end,1), path_2d_obs(end,2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', '终点');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
title(sprintf('2D带障碍物圆角 (半径=%.1f, 成功=%d)', fillet_radius, success_obs));
legend('Location', 'best');

fprintf('  带障碍物: 节点数 %d -> %d, 成功: %d\n', ...
    size(path_2d_obs, 1), size(path_smooth_obs, 1), success_obs);

%% 测试3: 3D路径的圆角处理
fprintf('\n========== 测试3: 3D路径圆角处理 ==========\n');

% 创建3D路径
path_3d = [
    0, 0, 0;
    10, 0, 5;
    10, 10, 5;
    20, 10, 10;
    20, 20, 15;
    30, 20, 20
];

obstacles_3d = [];
dim_3d = 3;
fillet_radius_3d = 2.5;

% 执行圆角处理
[path_smooth_3d, success_3d] = smoothPathWithFillets(path_3d, obstacles_3d, dim_3d, fillet_radius_3d, 15);

figure('Name', '3D路径圆角处理', 'Position', [900, 100, 800, 600]);

% 绘制原路径
plot3(path_3d(:,1), path_3d(:,2), path_3d(:,3), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '原路径');
hold on;

% 绘制圆角路径
plot3(path_smooth_3d(:,1), path_smooth_3d(:,2), path_smooth_3d(:,3), 'r-', 'LineWidth', 2, 'DisplayName', '圆角路径');

% 标记起点和终点
plot3(path_3d(1,1), path_3d(1,2), path_3d(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot3(path_3d(end,1), path_3d(end,2), path_3d(end,3), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', '终点');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('3D路径圆角 (半径=%.1f, 成功=%d)', fillet_radius_3d, success_3d));
legend('Location', 'best');
view(45, 30);

fprintf('  3D路径: 节点数 %d -> %d, 成功: %d\n', ...
    size(path_3d, 1), size(path_smooth_3d, 1), success_3d);

%% 测试4: 使用实际SC-RRT算法生成的路径
fprintf('\n========== 测试4: SC-RRT实际路径圆角处理 ==========\n');

try
    % 生成2D环境
    env = generate2DEnvironment([0 100 0 100], 15);
    
    % 运行SC-RRT算法
    fprintf('运行SC-RRT算法...\n');
    [path_rrt, tree_rrt, success_rrt, metrics_rrt] = SC_RRT_Basic(env, 3000, ...
        'Mode', 'adaptive', ...
        'StepSize', 3, ...
        'GoalThreshold', 5);
    
    if success_rrt && ~isempty(path_rrt)
        fprintf('  ✓ SC-RRT成功: 路径长度 %.2f, 节点数 %d\n', ...
            metrics_rrt.path_length, size(path_rrt, 1));
        
        % 可视化
        figure('Name', 'SC-RRT路径圆角效果', 'Position', [900, 600, 800, 600]);
        
        % 绘制障碍物
        for i = 1:size(env.obstacles, 1)
            theta = linspace(0, 2*pi, 50);
            x_circle = env.obstacles(i, 1) + env.obstacles(i, 3) * cos(theta);
            y_circle = env.obstacles(i, 2) + env.obstacles(i, 3) * sin(theta);
            fill(x_circle, y_circle, [0.8, 0.8, 0.8], 'EdgeColor', 'k');
            hold on;
        end
        
        % 绘制路径
        plot(path_rrt(:,1), path_rrt(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'SC-RRT路径(已圆角)');
        
        % 标记起点和终点
        plot(env.start(1), env.start(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
        plot(env.goal(1), env.goal(2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', '终点');
        
        grid on;
        axis equal;
        xlabel('X');
        ylabel('Y');
        title('SC-RRT路径(含圆角处理)');
        legend('Location', 'best');
    else
        fprintf('  ✗ SC-RRT失败\n');
    end
catch ME
    fprintf('  ⚠ SC-RRT测试异常: %s\n', ME.message);
    fprintf('     可能原因: generate2DEnvironment或SC_RRT_Basic不存在\n');
end

%% 总结
fprintf('\n========== 测试总结 ==========\n');
fprintf('所有测试完成。圆角平滑功能已集成到SC-RRT算法中。\n');
fprintf('主要特性:\n');
fprintf('  1. 自动识别拐点并进行圆角处理\n');
fprintf('  2. 使用贝塞尔曲线生成平滑过渡\n');
fprintf('  3. 集成碰撞检测，避免圆角与障碍物碰撞\n');
fprintf('  4. 自适应调整圆角半径，遇到碰撞时自动减小\n');
fprintf('  5. 支持2D和3D路径\n');
