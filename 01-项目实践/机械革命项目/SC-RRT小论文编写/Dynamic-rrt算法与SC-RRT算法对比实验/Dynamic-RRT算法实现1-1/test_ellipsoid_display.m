% =========================================================================
%     SC-RRT 椭球体动态显示测试
% =========================================================================
% 目的: 演示SC-RRT算法如何支持椭球体动态显示
%       将椭球体实时绘制在单独的窗口中，方便观察算法的搜索策略
% =========================================================================
clear; clc; close all;

%% =================== 环境配置 ===================
fprintf('\n========================================\n');
fprintf('SC-RRT 椭球体动态显示测试\n');
fprintf('========================================\n\n');

% 环境配置
config = struct();
config.dimension = '2D';
config.bounds = [0 1500 0 1500];
config.startPoint = [400 400];
config.goalPoint = [1100 1100];
config.numObstacles = 225;
config.obstacleRadius = 15;
config.seed = 42;
config.max_iterations = 10000;

fprintf('环境配置:\n');
fprintf('  空间: [%d %d] × [%d %d] mm\n', config.bounds);
fprintf('  起点: (%.0f, %.0f)\n', config.startPoint);
fprintf('  终点: (%.0f, %.0f)\n', config.goalPoint);
fprintf('  障碍物: %d 个 (半径 %d mm)\n', config.numObstacles, config.obstacleRadius);
fprintf('=========================================\n\n');

%% =================== 生成障碍物环境 ===================
fprintf('生成障碍物环境...\n');

% 检查是否有SC-RRT的generateObstacles
sc_rrt_path = '../SC-RRT独立算法实现/copilot-1.6';
if exist(fullfile(sc_rrt_path, 'generateObstacles.m'), 'file')
    addpath(sc_rrt_path);
    rng(config.seed);
    obstacles = generateObstacles(config.dimension, ...
                                 config.bounds, ...
                                 config.numObstacles, ...
                                 config.obstacleRadius, ...
                                 config.startPoint, ...
                                 config.goalPoint);
    fprintf('✓ 使用SC-RRT的障碍物生成器\n\n');
else
    fprintf('⚠ SC-RRT generateObstacles 未找到，使用Dynamic-RRT生成器\n\n');
    rng(config.seed);
    obstacles = GenerateObstacles(config.dimension, ...
                                 config.bounds, ...
                                 config.numObstacles, ...
                                 config.obstacleRadius, ...
                                 config.startPoint, ...
                                 config.goalPoint, ...
                                 config.seed);
end

fprintf('✓ 障碍物生成完成: %d 个\n\n', size(obstacles.circles, 1));

%% =================== 图1: 障碍物环境显示 ===================
fig_env = figure('Name', '环境配置 - SC-RRT椭球体显示测试', ...
                'Position', [100 100 800 700], ...
                'Color', 'w');

hold on; axis equal; grid on;
xlim(config.bounds(1:2));
ylim(config.bounds(3:4));
xlabel('X (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('测试环境 - 1500×1500mm, 225个障碍物', ...
      'FontSize', 14, 'FontWeight', 'bold');

% 绘制障碍物
for i = 1:size(obstacles.circles, 1)
    circle = obstacles.circles(i, :);
    rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                          2*circle(3), 2*circle(3)], ...
             'Curvature', [1 1], ...
             'FaceColor', [0.2 0.2 0.2], ...
             'EdgeColor', [0.2 0.2 0.2], ...
             'LineWidth', 0.5);
end

% 绘制起点
plot(config.startPoint(1), config.startPoint(2), 'go', ...
     'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2.5);
text(config.startPoint(1)-60, config.startPoint(2)-80, '起点', ...
     'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

% 绘制终点
plot(config.goalPoint(1), config.goalPoint(2), 'rs', ...
     'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2.5);
text(config.goalPoint(1)+60, config.goalPoint(2)+80, '终点', ...
     'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

set(gca, 'FontSize', 11);
fprintf('✓ 环境显示完成\n\n');

%% =================== 调用SC-RRT算法（带椭球体显示） ===================
fprintf('========================================\n');
fprintf('运行SC-RRT算法（支持椭球体动态显示）\n');
fprintf('========================================\n');
fprintf('椭球体显示规则:\n');
fprintf('  • 红色椭圆: 从起点到交汇点的采样约束\n');
fprintf('  • 蓝色椭圆: 从交汇点到终点的采样约束\n');
fprintf('  • 黄色菱形: 当前动态交汇点\n');
fprintf('========================================\n\n');

% 设置随机种子以保证可复现
rng(config.seed + 200);

% 调用SC_RRT_with_Ellipsoid_Display
try
    [tree, path, success, metrics, fig_ellipsoid] = SC_RRT_with_Ellipsoid_Display(...
        config.startPoint, ...
        config.goalPoint, ...
        config.bounds, ...
        obstacles, ...
        'MaxIterations', config.max_iterations, ...
        'Mode', 'pid', ...
        'UpdateInterval', 50);
    
    if success
        fprintf('\n✓ SC-RRT规划成功!\n');
        fprintf('  路径长度: %.2f mm\n', norm(path(end,:) - path(1,:)));
        fprintf('  总节点数: %d\n', metrics.nodeCount);
        fprintf('  迭代次数: %d\n', metrics.iterations);
    else
        fprintf('\n⚠ SC-RRT规划失败（在最大迭代次数内未找到路径）\n');
    end
    
catch ME
    fprintf('\n⚠ 运行发生错误: %s\n', ME.message);
    fprintf('  位置: %s\n', ME.stack(1).name);
    fprintf('\n  尝试使用简化版本的椭球体显示...\n\n');
    
    % 如果出错，显示简化信息
    fig_ellipsoid = figure('Name', 'SC-RRT 椭球体显示（简化版）', ...
        'Position', [950 100 900 800]);
    hold on; axis equal; grid on;
    xlim(config.bounds(1:2));
    ylim(config.bounds(3:4));
    xlabel('X (mm)', 'FontSize', 12);
    ylabel('Y (mm)', 'FontSize', 12);
    title('椭球体动态显示窗口', 'FontSize', 13, 'FontWeight', 'bold');
    
    % 绘制障碍物
    for i = 1:min(50, size(obstacles.circles, 1))
        circle = obstacles.circles(i, :);
        rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                              2*circle(3), 2*circle(3)], ...
                 'Curvature', [1 1], ...
                 'FaceColor', [0.3 0.3 0.3], ...
                 'EdgeColor', [0.3 0.3 0.3], ...
                 'LineWidth', 0.5);
    end
    
    plot(config.startPoint(1), config.startPoint(2), 'go', ...
         'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2.5);
    plot(config.goalPoint(1), config.goalPoint(2), 'rs', ...
         'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2.5);
    
    % 演示椭球体绘制（测试plotEllipsoid函数）
    fprintf('✓ 椭球体显示窗口已创建\n');
    fprintf('  绘制示意椭圆...\n\n');
    
    % 测试椭球体绘制
    meetPoint = (config.startPoint + config.goalPoint) / 2;
    cBestA = norm(meetPoint - config.startPoint) * 1.3;
    cBestB = norm(config.goalPoint - meetPoint) * 1.3;
    
    try
        h1 = plotEllipsoid(config.startPoint, meetPoint, cBestA, 2, [0.8 0.3 0.3]);
        h2 = plotEllipsoid(meetPoint, config.goalPoint, cBestB, 2, [0.3 0.3 0.8]);
        fprintf('✓ 示意椭圆已绘制\n');
    catch
        fprintf('⚠ 椭球体绘制出错\n');
    end
    
    drawnow;
end

%% =================== 功能说明 ===================
fprintf('\n========================================\n');
fprintf('功能总结\n');
fprintf('========================================\n\n');

fprintf('本测试脚本演示了以下功能：\n\n');
fprintf('1. 椭球体动态显示窗口\n');
fprintf('   • 独立创建一个新的图形窗口用于显示椭球体\n');
fprintf('   • 与算法执行窗口分离，方便观察\n\n');

fprintf('2. 双椭球约束可视化\n');
fprintf('   • 红色椭圆：起点→交汇点的采样椭圆\n');
fprintf('   • 蓝色椭圆：交汇点→终点的采样椭圆\n\n');

fprintf('3. 动态交汇点标记\n');
fprintf('   • 黄色菱形标记当前交汇点位置\n');
fprintf('   • 每个更新周期自动更新位置\n\n');

fprintf('4. 集成的辅助函数\n');
fprintf('   • plotEllipsoid.m: 椭球体绘制函数\n');
fprintf('   • calculateDualEllipsoidParams.m: 双椭球参数计算\n');
fprintf('   • calculatePotentialMeetPoint.m: 交汇点计算\n');
fprintf('   • SC_RRT_with_Ellipsoid_Display.m: 完整集成模块\n\n');

fprintf('========================================\n');
fprintf('使用建议：\n');
fprintf('========================================\n\n');

fprintf('1. 在compare_algorithms.m中启用椭球体显示:\n');
fprintf('   if strcmp(alg.type, ''sc_rrt'')\n');
fprintf('       enable_ellipsoid_display = true;\n');
fprintf('   end\n\n');

fprintf('2. 创建自定义椭球体显示窗口:\n');
fprintf('   fig_ellipsoid = figure(...);\n');
fprintf('   [tree, path, success] = SC_RRT_with_Ellipsoid_Display(...);\n\n');

fprintf('3. 在已有窗口中添加椭球体（见single_run_comparison.m）:\n');
fprintf('   使用plotEllipsoid函数直接绘制\n\n');

fprintf('========================================\n');
fprintf('测试完成！\n');
fprintf('========================================\n\n');

%% =================== 保存图片 ===================
fprintf('保存测试图片...\n');

% 创建保存目录
save_dir = 'paper_figures';
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% 保存环境图
saveas(fig_env, fullfile(save_dir, sprintf('ellipsoid_test_env_%s.png', timestamp)));
fprintf('✓ 环境图已保存\n');

% 保存椭球体显示窗口
if exist('fig_ellipsoid', 'var') && isvalid(fig_ellipsoid)
    saveas(fig_ellipsoid, fullfile(save_dir, sprintf('ellipsoid_display_%s.png', timestamp)));
    fprintf('✓ 椭球体显示窗口已保存\n');
end

fprintf('\n保存位置: %s/\n', save_dir);
fprintf('========================================\n\n');
