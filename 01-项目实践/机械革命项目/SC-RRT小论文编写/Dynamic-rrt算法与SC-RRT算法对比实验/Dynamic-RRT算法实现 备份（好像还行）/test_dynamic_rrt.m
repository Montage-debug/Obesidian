% =========================================================================
%          Dynamic RRT 算法测试 - 主程序
% =========================================================================
% 基于论文: Dynamic RRT: Fast Feasible Path Planning in Randomly 
%           Distributed Obstacle Environments (2023)
%
% 测试环境对应论文 Table 1
% =========================================================================
clear; clc; close all;

% =================== 环境配置 ===================
% 选择测试环境 (1-4)
env_idx = 1;

% 环境配置（对应论文Table 1）
env_configs = {
    % Environment 1: 2D固定障碍物（可复现）
    struct('dimension', '2D', ...
           'bounds', [0 1500 0 1500], ...
           'startPoint', [400 400], ...
           'goalPoint', [1100 1100], ...
           'numObstacles', 225, ...
           'obstacleRadius', 15, ...
           'seed', 42, ...
           'description', '2D Fixed Obstacles'),
    
    % Environment 2: 2D随机障碍物
    struct('dimension', '2D', ...
           'bounds', [0 1500 0 1500], ...
           'startPoint', [400 400], ...
           'goalPoint', [1100 1100], ...
           'numObstacles', 225, ...
           'obstacleRadius', 15, ...
           'seed', NaN, ...
           'description', '2D Random Obstacles'),
    
    % Environment 3: 3D固定障碍物（可复现）
    struct('dimension', '3D', ...
           'bounds', [0 1500 0 1500 0 1500], ...
           'startPoint', [1 1 1], ...
           'goalPoint', [1100 1100 1100], ...
           'numObstacles', 400, ...
           'obstacleRadius', 18, ...
           'seed', 123, ...
           'description', '3D Fixed Obstacles'),
    
    % Environment 4: 3D随机障碍物
    struct('dimension', '3D', ...
           'bounds', [0 1500 0 1500 0 1500], ...
           'startPoint', [1 1 1], ...
           'goalPoint', [1100 1100 1100], ...
           'numObstacles', 400, ...
           'obstacleRadius', 18, ...
           'seed', NaN, ...
           'description', '3D Random Obstacles')
};

config = env_configs{env_idx};

% =================== 算法参数 ===================
% Interval参数（论文关键参数，影响速度-路径质量折中）
% 论文测试值: 3, 4, 6, 8, 10, 20
interval = 4;  % 论文推荐值

% 其他参数
params = struct();
params.MaxIterations = 10000;
params.Interval = interval;
params.ParetoProb = 0.1;  % 非Pareto节点选择概率
params.EnableVisualization = true;  % 启用可视化
params.VisualizationInterval = 100;  % 可视化更新间隔

fprintf('\n========================================\n');
fprintf('Dynamic RRT 算法测试\n');
fprintf('========================================\n');
fprintf('环境: %s\n', config.description);
fprintf('维度: %s\n', config.dimension);
fprintf('障碍物数量: %d\n', config.numObstacles);
fprintf('Interval: %d\n', interval);
fprintf('========================================\n\n');

% =================== 生成障碍物 ===================
fprintf('生成障碍物...\n');
obstacles = GenerateObstacles(config.dimension, config.bounds, ...
                              config.numObstacles, config.obstacleRadius, ...
                              config.startPoint, config.goalPoint, config.seed);

if strcmp(config.dimension, '2D')
    fprintf('实际生成: %d 个圆形障碍物\n', size(obstacles.circles, 1));
else
    fprintf('实际生成: %d 个球形障碍物\n', size(obstacles.spheres, 1));
end

% =================== 运行Dynamic RRT ===================
fprintf('\n开始运行 Dynamic RRT...\n');

[tree, path, success, metrics] = DynamicRRT(config.startPoint, config.goalPoint, ...
                                             config.bounds, obstacles, ...
                                             'MaxIterations', params.MaxIterations, ...
                                             'Interval', params.Interval, ...
                                             'ParetoProb', params.ParetoProb, ...
                                             'EnableVisualization', params.EnableVisualization, ...
                                             'VisualizationInterval', params.VisualizationInterval);

% =================== 结果输出 ===================
fprintf('\n========================================\n');
fprintf('测试结果\n');
fprintf('========================================\n');

if success
    fprintf('✓ 路径规划成功!\n');
    fprintf('收敛时间: %.4f 秒\n', metrics.convergenceTime);
    fprintf('路径长度: %.2f\n', metrics.pathLength);
    fprintf('迭代次数: %d\n', metrics.iterations);
    fprintf('树节点数: %d\n', metrics.nodeCount);
    fprintf('\n与论文对比 (Interval=%d):\n', interval);
    fprintf('  论文参考值 (2D, interval=4): ~0.034s, ~1157.4\n');
    fprintf('  当前结果: %.4fs, %.2f\n', metrics.convergenceTime, metrics.pathLength);
else
    fprintf('✗ 路径规划失败\n');
    fprintf('达到最大迭代次数: %d\n', params.MaxIterations);
end

fprintf('========================================\n');

% =================== 可视化最终结果 ===================
if success && params.EnableVisualization
    figure('Name', 'Dynamic RRT - 最终结果', 'Position', [100 100 900 800]);
    
    if strcmp(config.dimension, '2D')
        hold on; grid on; axis equal;
        xlim([config.bounds(1) config.bounds(2)]);
        ylim([config.bounds(3) config.bounds(4)]);
        
        % 绘制障碍物
        for i = 1:size(obstacles.circles, 1)
            rectangle('Position', [obstacles.circles(i,1:2)-obstacles.circles(i,3), ...
                     2*obstacles.circles(i,3), 2*obstacles.circles(i,3)], ...
                     'Curvature', [1 1], 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
        end
        
        % 绘制树
        for i = 2:tree.count
            parent = tree.parents(i);
            plot([tree.nodes(parent,1) tree.nodes(i,1)], ...
                 [tree.nodes(parent,2) tree.nodes(i,2)], ...
                 'b-', 'LineWidth', 0.3, 'Color', [0.7 0.7 1]);
        end
        
        % 绘制路径
        plot(path(:,1), path(:,2), 'r-', 'LineWidth', 3);
        
        % 绘制起点和终点
        plot(config.startPoint(1), config.startPoint(2), 'go', ...
             'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        plot(config.goalPoint(1), config.goalPoint(2), 'ro', ...
             'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        
        title(sprintf('Dynamic RRT (Interval=%d) - Time: %.4fs, Length: %.2f', ...
                      interval, metrics.convergenceTime, metrics.pathLength));
        xlabel('X'); ylabel('Y');
        legend('障碍物', '搜索树', '最终路径', '起点', '终点', 'Location', 'best');
        
    else
        % 3D可视化
        hold on; grid on; axis equal;
        xlim([config.bounds(1) config.bounds(2)]);
        ylim([config.bounds(3) config.bounds(4)]);
        zlim([config.bounds(5) config.bounds(6)]);
        view(3);
        
        % 绘制障碍物
        for i = 1:size(obstacles.spheres, 1)
            [X,Y,Z] = sphere(15);
            surf(X*obstacles.spheres(i,4)+obstacles.spheres(i,1), ...
                 Y*obstacles.spheres(i,4)+obstacles.spheres(i,2), ...
                 Z*obstacles.spheres(i,4)+obstacles.spheres(i,3), ...
                 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
        end
        
        % 绘制树
        for i = 2:tree.count
            parent = tree.parents(i);
            plot3([tree.nodes(parent,1) tree.nodes(i,1)], ...
                  [tree.nodes(parent,2) tree.nodes(i,2)], ...
                  [tree.nodes(parent,3) tree.nodes(i,3)], ...
                  'b-', 'LineWidth', 0.3, 'Color', [0.7 0.7 1]);
        end
        
        % 绘制路径
        plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 3);
        
        % 绘制起点和终点
        plot3(config.startPoint(1), config.startPoint(2), config.startPoint(3), ...
              'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        plot3(config.goalPoint(1), config.goalPoint(2), config.goalPoint(3), ...
              'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        
        title(sprintf('Dynamic RRT 3D (Interval=%d) - Time: %.4fs, Length: %.2f', ...
                      interval, metrics.convergenceTime, metrics.pathLength));
        xlabel('X'); ylabel('Y'); zlabel('Z');
    end
end
