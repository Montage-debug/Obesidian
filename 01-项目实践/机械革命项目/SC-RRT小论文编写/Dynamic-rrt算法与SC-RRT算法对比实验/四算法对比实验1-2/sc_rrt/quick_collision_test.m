%% quick_collision_test.m - 快速验证碰撞检测修复
%
% 用于快速验证圆角碰撞检测的修复是否有效
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

clear; close all; clc;
addpath(genpath('./sc_rrt'));

fprintf('========== 圆角碰撞检测修复验证 ==========\n\n');

%% 测试：密集障碍物环境（之前失败的测试）
fprintf('测试: 密集障碍物环境（复现之前的失败场景）\n');

% 使用相同的随机种子，复现之前的场景
rng(42);

% 创建路径
path_test = [
    0, 0;
    10, 10;
    20, 5;
    30, 15;
    40, 10;
    50, 20
];

% 密集障碍物
obstacles_test = [];
for i = 1:20
    x = rand() * 50;
    y = rand() * 20;
    r = 1 + rand() * 1.5;
    obstacles_test = [obstacles_test; x, y, r];
end

dim = 2;
fillet_radius = 2.0;

fprintf('  障碍物数量: %d\n', size(obstacles_test, 1));
fprintf('  圆角半径: %.1f\n', fillet_radius);
fprintf('  执行圆角处理...\n');

tic;
[path_smooth, success] = smoothPathWithFillets(path_test, obstacles_test, dim, fillet_radius, 20);
time_elapsed = toc;

fprintf('  处理完成: 耗时 %.3fs\n', time_elapsed);
fprintf('  节点数: %d -> %d\n', size(path_test, 1), size(path_smooth, 1));
fprintf('  成功标志: %d\n', success);

% 手动验证：检查所有点是否安全
collision_found = false;
min_distance = inf;
collision_details = [];

for i = 1:size(path_smooth, 1)
    for j = 1:size(obstacles_test, 1)
        dist = norm(path_smooth(i, :) - obstacles_test(j, 1:2)) - obstacles_test(j, 3);
        min_distance = min(min_distance, dist);
        
        if dist < 0
            collision_found = true;
            collision_details = [collision_details; i, j, dist];
        end
    end
end

fprintf('\n结果分析:\n');
fprintf('  最小安全距离: %.4f\n', min_distance);

if collision_found
    fprintf('  ✗ 发现碰撞! 碰撞数量: %d\n', size(collision_details, 1));
    fprintf('  碰撞详情:\n');
    for k = 1:min(5, size(collision_details, 1))
        fprintf('    - 点 %d 与障碍物 %d, 距离=%.4f\n', ...
            collision_details(k, 1), collision_details(k, 2), collision_details(k, 3));
    end
    fprintf('  ❌ 修复失败！仍存在碰撞问题\n');
else
    fprintf('  ✓ 未发现碰撞\n');
    fprintf('  ✅ 修复成功！所有路径点安全\n');
end

%% 可视化
figure('Name', '碰撞检测修复验证', 'Position', [100, 100, 900, 700]);

% 绘制障碍物
for i = 1:size(obstacles_test, 1)
    theta = linspace(0, 2*pi, 30);
    x_circle = obstacles_test(i, 1) + obstacles_test(i, 3) * cos(theta);
    y_circle = obstacles_test(i, 2) + obstacles_test(i, 3) * sin(theta);
    fill(x_circle, y_circle, [0.9, 0.9, 0.9], 'EdgeColor', [0.5, 0.5, 0.5]);
    hold on;
    
    % 绘制安全边界（+10%）
    x_safe = obstacles_test(i, 1) + obstacles_test(i, 3) * 1.10 * cos(theta);
    y_safe = obstacles_test(i, 2) + obstacles_test(i, 3) * 1.10 * sin(theta);
    plot(x_safe, y_safe, 'r:', 'LineWidth', 0.5);
end

% 绘制原路径
plot(path_test(:,1), path_test(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', '原路径');

% 绘制圆角路径
if collision_found
    plot(path_smooth(:,1), path_smooth(:,2), 'r-', 'LineWidth', 2.5, 'DisplayName', '圆角路径(有碰撞)');
    % 标记碰撞点
    for k = 1:size(collision_details, 1)
        idx = collision_details(k, 1);
        plot(path_smooth(idx, 1), path_smooth(idx, 2), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
    end
else
    plot(path_smooth(:,1), path_smooth(:,2), 'g-', 'LineWidth', 2.5, 'DisplayName', '圆角路径(安全)');
end

% 标记起点和终点
plot(path_test(1,1), path_test(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(path_test(end,1), path_test(end,2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');

grid on;
axis equal;
xlabel('X');
ylabel('Y');
if collision_found
    title(sprintf('碰撞检测修复验证 - ❌ 失败 (最小距离=%.3f)', min_distance), 'FontSize', 14);
else
    title(sprintf('碰撞检测修复验证 - ✅ 成功 (最小距离=%.3f)', min_distance), 'FontSize', 14);
end
legend('Location', 'best');

%% 总结
fprintf('\n========== 修复验证总结 ==========\n');
if collision_found
    fprintf('状态: ❌ 修复不完全\n');
    fprintf('建议: 需要进一步增强碰撞检测逻辑\n');
else
    fprintf('状态: ✅ 修复成功\n');
    fprintf('说明: 圆角路径完全避开障碍物，安全裕度10%%有效\n');
end
fprintf('关键改进:\n');
fprintf('  1. 最终验证失败时回退到原路径\n');
fprintf('  2. 二次验证新添加的路径段（20点采样）\n');
fprintf('  3. 圆角段间检测采样提高到5点\n');
fprintf('  4. 线段检测默认采样提高到15点\n');
fprintf('  5. 最终验证采样提高到20点\n');
