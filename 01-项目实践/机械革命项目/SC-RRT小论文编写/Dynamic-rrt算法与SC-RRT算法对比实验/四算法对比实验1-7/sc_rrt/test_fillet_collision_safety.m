%% test_fillet_collision_safety.m - 测试圆角碰撞检测的安全性
% 
% 功能：专门测试圆角处理在障碍物附近的碰撞检测
% 
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

clear; close all; clc;
addpath(genpath('./sc_rrt'));

fprintf('========== 圆角碰撞检测安全性测试 ==========\n\n');

%% 测试1: 路径穿过障碍物边缘的圆角处理
fprintf('测试1: 路径靠近障碍物边缘\n');

% 创建一个路径，拐点非常靠近障碍物
path_danger = [
    0, 0;
    8, 5;    % 拐点靠近障碍物
    8, 15;
    20, 15
];

% 障碍物在拐点附近
obstacles_danger = [
    10, 8, 3;  % 圆心(10, 8)，半径3，距离拐点约2.5
];

dim = 2;
fillet_radius = 2.0;

% 执行圆角处理
fprintf('  执行圆角处理（半径=%.1f）...\n', fillet_radius);
[path_smooth_1, success_1] = smoothPathWithFillets(path_danger, obstacles_danger, dim, fillet_radius, 20);

% 可视化
figure('Name', '测试1: 危险拐点圆角处理', 'Position', [100, 100, 800, 600]);

% 绘制障碍物
theta = linspace(0, 2*pi, 50);
x_circle = obstacles_danger(1) + obstacles_danger(3) * cos(theta);
y_circle = obstacles_danger(2) + obstacles_danger(3) * sin(theta);
fill(x_circle, y_circle, [1, 0.7, 0.7], 'EdgeColor', 'r', 'LineWidth', 2, 'DisplayName', '障碍物');
hold on;

% 绘制安全区域（障碍物+10%安全裕度）
x_circle_safe = obstacles_danger(1) + obstacles_danger(3) * 1.10 * cos(theta);
y_circle_safe = obstacles_danger(2) + obstacles_danger(3) * 1.10 * sin(theta);
plot(x_circle_safe, y_circle_safe, 'r--', 'LineWidth', 1.5, 'DisplayName', '安全边界(+10%)');

% 绘制原路径
plot(path_danger(:,1), path_danger(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '原路径');

% 绘制圆角路径
plot(path_smooth_1(:,1), path_smooth_1(:,2), 'g-', 'LineWidth', 2, 'DisplayName', '圆角路径');

% 标记拐点
plot(path_danger(2,1), path_danger(2,2), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'DisplayName', '危险拐点');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
title(sprintf('测试1: 拐点靠近障碍物 (成功=%d)', success_1));
legend('Location', 'best');

fprintf('  结果: 成功=%d, 节点数 %d -> %d\n', success_1, size(path_danger, 1), size(path_smooth_1, 1));

% 检查路径是否安全
min_dist = inf;
for i = 1:size(path_smooth_1, 1)
    dist = norm(path_smooth_1(i, :) - obstacles_danger(1:2)) - obstacles_danger(3);
    min_dist = min(min_dist, dist);
end
fprintf('  最小距离: %.4f (应该 >= %.4f)\n', min_dist, obstacles_danger(3) * 0.10);

if min_dist >= 0
    fprintf('  ✓ 路径安全！\n');
else
    fprintf('  ✗ 警告：路径与障碍物碰撞！\n');
end

%% 测试2: 多个障碍物包围的路径
fprintf('\n测试2: 多障碍物环境\n');

path_multi = [
    0, 0;
    10, 5;
    10, 15;
    20, 15;
    20, 25;
    30, 25
];

obstacles_multi = [
    8, 8, 2.5;
    12, 12, 2;
    18, 18, 2.5;
    22, 22, 2
];

fillet_radius_2 = 2.5;

fprintf('  执行圆角处理（半径=%.1f）...\n', fillet_radius_2);
[path_smooth_2, success_2] = smoothPathWithFillets(path_multi, obstacles_multi, dim, fillet_radius_2, 20);

% 可视化
figure('Name', '测试2: 多障碍物环境', 'Position', [920, 100, 800, 600]);

% 绘制所有障碍物
for i = 1:size(obstacles_multi, 1)
    theta = linspace(0, 2*pi, 50);
    x_circle = obstacles_multi(i, 1) + obstacles_multi(i, 3) * cos(theta);
    y_circle = obstacles_multi(i, 2) + obstacles_multi(i, 3) * sin(theta);
    fill(x_circle, y_circle, [1, 0.7, 0.7], 'EdgeColor', 'r', 'LineWidth', 1.5);
    hold on;
    
    % 安全边界
    x_circle_safe = obstacles_multi(i, 1) + obstacles_multi(i, 3) * 1.10 * cos(theta);
    y_circle_safe = obstacles_multi(i, 2) + obstacles_multi(i, 3) * 1.10 * sin(theta);
    plot(x_circle_safe, y_circle_safe, 'r--', 'LineWidth', 1);
end

% 绘制路径
plot(path_multi(:,1), path_multi(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '原路径');
plot(path_smooth_2(:,1), path_smooth_2(:,2), 'g-', 'LineWidth', 2, 'DisplayName', '圆角路径');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
title(sprintf('测试2: 多障碍物环境 (成功=%d)', success_2));
legend('Location', 'best');

fprintf('  结果: 成功=%d, 节点数 %d -> %d\n', success_2, size(path_multi, 1), size(path_smooth_2, 1));

% 检查每段路径的安全距离
all_safe = true;
for i = 1:size(path_smooth_2, 1)
    for j = 1:size(obstacles_multi, 1)
        dist = norm(path_smooth_2(i, :) - obstacles_multi(j, 1:2)) - obstacles_multi(j, 3);
        if dist < 0
            fprintf('  ✗ 警告：点 %d 与障碍物 %d 碰撞！距离=%.4f\n', i, j, dist);
            all_safe = false;
        end
    end
end

if all_safe
    fprintf('  ✓ 所有路径点安全！\n');
end

%% 测试3: 极端情况 - 圆角不可能避障
fprintf('\n测试3: 不可避障情况\n');

path_impossible = [
    0, 0;
    10, 10;  % 拐点在障碍物内部
    20, 0
];

obstacles_impossible = [
    10, 10, 5;  % 障碍物覆盖拐点
];

fillet_radius_3 = 3.0;

fprintf('  执行圆角处理（半径=%.1f）...\n', fillet_radius_3);
[path_smooth_3, success_3] = smoothPathWithFillets(path_impossible, obstacles_impossible, dim, fillet_radius_3, 20);

% 可视化
figure('Name', '测试3: 不可避障情况', 'Position', [100, 700, 800, 600]);

% 绘制障碍物
theta = linspace(0, 2*pi, 50);
x_circle = obstacles_impossible(1) + obstacles_impossible(3) * cos(theta);
y_circle = obstacles_impossible(2) + obstacles_impossible(3) * sin(theta);
fill(x_circle, y_circle, [1, 0.7, 0.7], 'EdgeColor', 'r', 'LineWidth', 2, 'DisplayName', '障碍物');
hold on;

% 绘制路径
plot(path_impossible(:,1), path_impossible(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '原路径');
plot(path_smooth_3(:,1), path_smooth_3(:,2), 'g-', 'LineWidth', 2, 'DisplayName', '处理后路径');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
title(sprintf('测试3: 不可避障 (成功=%d, 应保留原拐点)', success_3));
legend('Location', 'best');

fprintf('  结果: 成功=%d, 节点数 %d -> %d\n', success_3, size(path_impossible, 1), size(path_smooth_3, 1));

if success_3 == 0
    fprintf('  ✓ 正确识别无法圆角，保留原拐点\n');
else
    fprintf('  ⚠ 注意：标记为成功但可能有碰撞\n');
end

%% 测试4: 应力测试 - 密集障碍物
fprintf('\n测试4: 密集障碍物环境\n');

% 创建复杂路径
path_dense = [
    0, 0;
    10, 10;
    20, 5;
    30, 15;
    40, 10;
    50, 20
];

% 在路径附近密集放置障碍物
rng(42);  % 固定随机种子
obstacles_dense = [];
for i = 1:20
    x = rand() * 50;
    y = rand() * 20;
    r = 1 + rand() * 1.5;
    obstacles_dense = [obstacles_dense; x, y, r];
end

fillet_radius_4 = 2.0;

fprintf('  执行圆角处理（半径=%.1f, 障碍物数=%d）...\n', fillet_radius_4, size(obstacles_dense, 1));
tic;
[path_smooth_4, success_4] = smoothPathWithFillets(path_dense, obstacles_dense, dim, fillet_radius_4, 20);
time_4 = toc;

% 可视化
figure('Name', '测试4: 密集障碍物', 'Position', [920, 700, 800, 600]);

% 绘制障碍物
for i = 1:size(obstacles_dense, 1)
    theta = linspace(0, 2*pi, 30);
    x_circle = obstacles_dense(i, 1) + obstacles_dense(i, 3) * cos(theta);
    y_circle = obstacles_dense(i, 2) + obstacles_dense(i, 3) * sin(theta);
    fill(x_circle, y_circle, [0.9, 0.9, 0.9], 'EdgeColor', [0.5, 0.5, 0.5]);
    hold on;
end

% 绘制路径
plot(path_dense(:,1), path_dense(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '原路径');
plot(path_smooth_4(:,1), path_smooth_4(:,2), 'r-', 'LineWidth', 2.5, 'DisplayName', '圆角路径');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
title(sprintf('测试4: 密集障碍物 (成功=%d, 耗时=%.3fs)', success_4, time_4));
legend('Location', 'best');

fprintf('  结果: 成功=%d, 节点数 %d -> %d, 耗时=%.3fs\n', ...
    success_4, size(path_dense, 1), size(path_smooth_4, 1), time_4);

% 全路径碰撞检测
collision_found = false;
for i = 1:size(path_smooth_4, 1)
    for j = 1:size(obstacles_dense, 1)
        dist = norm(path_smooth_4(i, :) - obstacles_dense(j, 1:2)) - obstacles_dense(j, 3);
        if dist < 0
            fprintf('  ✗ 碰撞：点 %d 与障碍物 %d, 距离=%.4f\n', i, j, dist);
            collision_found = true;
        end
    end
end

if ~collision_found
    fprintf('  ✓ 所有路径点安全！\n');
end

%% 总结
fprintf('\n========== 测试总结 ==========\n');
fprintf('圆角碰撞检测安全性验证完成。\n');
fprintf('关键改进:\n');
fprintf('  1. 采样密度提高到20点/圆角\n');
fprintf('  2. 安全裕度提高到10%%\n');
fprintf('  3. 增加线段碰撞检测\n');
fprintf('  4. 增加最终全路径验证\n');
fprintf('  5. 碰撞时自动保留原拐点\n');
fprintf('\n建议:\n');
fprintf('  - 如果环境复杂，可增加采样密度到30点\n');
fprintf('  - 如果需要更保守，可提高安全裕度到15%%\n');
fprintf('  - 对于实时应用，可适当降低采样密度到10-15点\n');
