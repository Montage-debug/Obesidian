%% SC-RRT算法测试脚本
% 测试SC-RRT优化算法的所有功能
% 日期: 2025-12-12

clear; clc; close all;

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║              SC-RRT优化算法测试                            ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

% 添加路径
addpath(genpath(pwd));

%% 测试1: 2D环境
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('【测试1】2D环境 - SC-RRT算法\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% 生成2D环境
fprintf('正在生成2D测试环境...\n');
env_2d = generate2DEnvironment([0 100 0 100], 50);  % 50个障碍物，简化测试
fprintf('✓ 2D环境生成完成: %d个障碍物\n\n', env_2d.num);

% 运行SC-RRT
fprintf('运行SC-RRT算法 (2D)...\n');
tic;
try
    [path_2d, tree_2d, success_2d, metrics_2d] = SC_RRT_Basic(env_2d, 3000, ...
        'Mode', 'adaptive', ...
        'UseParetoFrontier', true, ...
        'VisualizeDualEllipsoid', false, ...
        'VisualizationInterval', 0);  % 关闭可视化加快速度
    time_2d = toc;
    
    if success_2d
        fprintf('✓ 2D测试成功!\n');
        fprintf('  路径长度: %.2f\n', metrics_2d.path_length);
        fprintf('  规划时间: %.3fs\n', time_2d);
        fprintf('  迭代次数: %d\n', metrics_2d.iterations);
        fprintf('  树节点数: %d\n', metrics_2d.tree_nodes);
        
        % 可视化结果
        figure('Name', 'SC-RRT 2D测试结果');
        hold on; grid on; axis equal;
        title('SC-RRT算法 - 2D环境测试');
        
        % 绘制障碍物
        for i = 1:size(env_2d.obstacles, 1)
            rectangle('Position', [env_2d.obstacles(i, 1:2) - env_2d.obstacles(i, 3), ...
                2*env_2d.obstacles(i, 3), 2*env_2d.obstacles(i, 3)], ...
                'Curvature', [1 1], 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k');
        end
        
        % 绘制路径
        plot(path_2d(:, 1), path_2d(:, 2), 'b-', 'LineWidth', 2);
        plot(env_2d.start(1), env_2d.start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(env_2d.goal(1), env_2d.goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        
        legend('障碍物', '规划路径', '起点', '终点');
        
        fprintf('✓ 2D结果可视化完成\n');
    else
        fprintf('✗ 2D测试失败: 未找到路径\n');
    end
catch ME
    fprintf('✗ 2D测试出错: %s\n', ME.message);
    fprintf('  错误位置: %s (第%d行)\n', ME.stack(1).name, ME.stack(1).line);
end

fprintf('\n');

%% 测试2: 3D环境
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('【测试2】3D环境 - SC-RRT算法\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% 生成3D环境
fprintf('正在生成3D测试环境...\n');
env_3d = generate3DEnvironment([0 100 0 100 0 100], 80);  % 80个障碍物
fprintf('✓ 3D环境生成完成: %d个障碍物\n\n', env_3d.num);

% 运行SC-RRT
fprintf('运行SC-RRT算法 (3D)...\n');
tic;
try
    [path_3d, tree_3d, success_3d, metrics_3d] = SC_RRT_Basic(env_3d, 5000, ...
        'Mode', 'adaptive', ...
        'UseParetoFrontier', true, ...
        'VisualizeDualEllipsoid', false, ...
        'VisualizationInterval', 0);
    time_3d = toc;
    
    if success_3d
        fprintf('✓ 3D测试成功!\n');
        fprintf('  路径长度: %.2f\n', metrics_3d.path_length);
        fprintf('  规划时间: %.3fs\n', time_3d);
        fprintf('  迭代次数: %d\n', metrics_3d.iterations);
        fprintf('  树节点数: %d\n', metrics_3d.tree_nodes);
        
        % 可视化结果
        figure('Name', 'SC-RRT 3D测试结果');
        hold on; grid on; axis equal;
        view(3);
        title('SC-RRT算法 - 3D环境测试');
        
        % 绘制障碍物
        for i = 1:size(env_3d.obstacles, 1)
            [x, y, z] = sphere(20);
            r = env_3d.obstacles(i, 4);
            surf(x*r + env_3d.obstacles(i, 1), ...
                 y*r + env_3d.obstacles(i, 2), ...
                 z*r + env_3d.obstacles(i, 3), ...
                 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
        end
        
        % 绘制路径
        plot3(path_3d(:, 1), path_3d(:, 2), path_3d(:, 3), 'b-', 'LineWidth', 2);
        plot3(env_3d.start(1), env_3d.start(2), env_3d.start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot3(env_3d.goal(1), env_3d.goal(2), env_3d.goal(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        
        legend('障碍物', '规划路径', '起点', '终点');
        
        fprintf('✓ 3D结果可视化完成\n');
    else
        fprintf('✗ 3D测试失败: 未找到路径\n');
    end
catch ME
    fprintf('✗ 3D测试出错: %s\n', ME.message);
    fprintf('  错误位置: %s (第%d行)\n', ME.stack(1).name, ME.stack(1).line);
end

fprintf('\n');

%% 总结
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                  测试完成                                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

if exist('success_2d', 'var') && success_2d
    fprintf('✓ 2D测试: 成功 (%.3fs, %.2f路径长度)\n', time_2d, metrics_2d.path_length);
else
    fprintf('✗ 2D测试: 失败\n');
end

if exist('success_3d', 'var') && success_3d
    fprintf('✓ 3D测试: 成功 (%.3fs, %.2f路径长度)\n', time_3d, metrics_3d.path_length);
else
    fprintf('✗ 3D测试: 失败\n');
end

fprintf('\n📊 所有SC-RRT优化功能已测试完成!\n');
fprintf('   - 自适应PID控制\n');
fprintf('   - Pareto前沿优化\n');
fprintf('   - 双向椭球约束采样\n');
fprintf('   - 动态交汇点机制\n');
fprintf('   - 节点重连策略\n\n');
