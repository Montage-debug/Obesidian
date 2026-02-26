function [tree, path, success, metrics] = DynamicRRT(startPoint, goalPoint, bounds, obstacles, varargin)
% DynamicRRT - Dynamic RRT算法实现 (论文复现版本)
%
% 论文: Dynamic RRT: Fast Feasible Path Planning in Randomly Distributed 
%       Obstacle Environments (2023)
%
% 核心特性:
%   1. Informed subset采样: 使用F_hat(c)估计构建超椭球采样空间
%   2. Pareto dominance动态规划: 周期性选择新起点分解问题
%   3. 可调interval参数: 平衡收敛速度与路径质量
%
% 输入参数:
%   startPoint  - 起始点 [1 x m]
%   goalPoint   - 目标点 [1 x m]
%   bounds      - 规划空间边界 [1 x 2m]
%   obstacles   - 障碍物结构体 (.circles或.spheres)
%   varargin    - 可选参数:
%     'MaxIterations'    - 最大迭代次数 (默认10000)
%     'Interval'         - Pareto选择间隔 (默认4)
%     'ParetoProb'       - 非Pareto节点选择概率 (默认0.1)
%     'StepSize'         - 扩展步长 (默认自动)
%     'GoalThreshold'    - 目标阈值 (默认自动)
%     'EnableVisualization' - 启用可视化 (默认false)
%     'VisualizationInterval' - 可视化间隔 (默认50)
%
% 输出:
%   tree     - RRT树结构
%   path     - 规划路径
%   success  - 是否成功
%   metrics  - 性能指标结构体

% ========== 参数解析 ==========
p = inputParser;
addParameter(p, 'MaxIterations', 10000, @isnumeric);
addParameter(p, 'Interval', 4, @isnumeric);
addParameter(p, 'ParetoProb', 0.1, @isnumeric);
addParameter(p, 'StepSize', [], @isnumeric);
addParameter(p, 'GoalThreshold', [], @isnumeric);
addParameter(p, 'EnableVisualization', false, @islogical);
addParameter(p, 'VisualizationInterval', 50, @isnumeric);
parse(p, varargin{:});

maxIterations = p.Results.MaxIterations;
interval = p.Results.Interval;
paretoProb = p.Results.ParetoProb;
enableVisualization = p.Results.EnableVisualization;
visualizationInterval = p.Results.VisualizationInterval;

% ========== 维度相关参数 ==========
m = length(startPoint);

if isempty(p.Results.StepSize)
    if m == 2
        stepSize = 30;
    else
        stepSize = 40;
    end
else
    stepSize = p.Results.StepSize;
end

if isempty(p.Results.GoalThreshold)
    if m == 2
        goalThreshold = 20;
    else
        goalThreshold = 30;
    end
else
    goalThreshold = p.Results.GoalThreshold;
end

% ========== 算法初始化 ==========
fprintf('\n========== Dynamic RRT算法执行 (%dD) ==========\n', m);
fprintf('Interval: %d\n', interval);
fprintf('Pareto非最优概率: %.2f\n', paretoProb);
fprintf('步长: %.1f, 目标阈值: %.1f\n', stepSize, goalThreshold);
fprintf('==========================================\n\n');

tic;

% 初始化树结构
tree = struct();
tree.nodes = [startPoint];
tree.parents = [0];
tree.costs = [0];
tree.children = {[]};
tree.count = 1;

% 初始化当前起点
currentStart = startPoint;
currentStartIdx = 1;

% 迭代计数器
counter = 0;
success = false;
convergenceTime = NaN;
firstSolutionIter = NaN;
cIdx = 1;

% 可视化初始化
if enableVisualization
    fig = figure('Name', 'Dynamic RRT', 'Position', [100 100 800 800]);
    if m == 2
        hold on; grid on; axis equal;
        xlim([bounds(1) bounds(2)]);
        ylim([bounds(3) bounds(4)]);
        if isfield(obstacles, 'circles')
            for i = 1:size(obstacles.circles, 1)
                rectangle('Position', [obstacles.circles(i,1:2)-obstacles.circles(i,3), ...
                         2*obstacles.circles(i,3), 2*obstacles.circles(i,3)], ...
                         'Curvature', [1 1], 'FaceColor', [0.7 0.7 0.7]);
            end
        end
        plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot(goalPoint(1), goalPoint(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    else
        hold on; grid on; axis equal;
        xlim([bounds(1) bounds(2)]);
        ylim([bounds(3) bounds(4)]);
        zlim([bounds(5) bounds(6)]);
        view(3);
        if isfield(obstacles, 'spheres')
            for i = 1:size(obstacles.spheres, 1)
                [X,Y,Z] = sphere(20);
                surf(X*obstacles.spheres(i,4)+obstacles.spheres(i,1), ...
                     Y*obstacles.spheres(i,4)+obstacles.spheres(i,2), ...
                     Z*obstacles.spheres(i,4)+obstacles.spheres(i,3), ...
                     'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
            end
        end
        plot3(startPoint(1), startPoint(2), startPoint(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        plot3(goalPoint(1), goalPoint(2), goalPoint(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end
end

% ========== 主循环 ==========
for iter = 1:maxIterations
    % 1. 找到离goal最近的节点c
    distances = vecnorm(tree.nodes(1:tree.count,:) - repmat(goalPoint, tree.count, 1), 2, 2);
    [minDist, cIdx] = min(distances);
    
    % 检查是否到达goal
    if minDist <= goalThreshold
        if ~success
            success = true;
            convergenceTime = toc;
            firstSolutionIter = iter;
            fprintf('首次收敛! 迭代: %d, 时间: %.4f秒\n', iter, convergenceTime);
        end
        break;
    end
    
    % 2. 计算F_hat(c)
    c_node = tree.nodes(cIdx, :);
    c_cost = tree.costs(cIdx);
    F_hat = CalCostHat(currentStart, goalPoint, c_node, c_cost, tree, cIdx);
    
    % 3. 在informed subset内采样
    x_rand = SampleEllipsoid(currentStart, goalPoint, bounds, F_hat, m);
    
    % 4. 找到最近节点并扩展
    x_nearest_idx = Nearest(tree, x_rand);
    x_nearest = tree.nodes(x_nearest_idx, :);
    x_new = Steer(x_nearest, x_rand, stepSize);
    
    % 5. 碰撞检测
    if ~CheckCollision(x_nearest, x_new, obstacles, m)
        tree.count = tree.count + 1;
        tree.nodes(tree.count, :) = x_new;
        tree.parents(tree.count) = x_nearest_idx;
        new_cost = tree.costs(x_nearest_idx) + norm(x_new - x_nearest);
        tree.costs(tree.count) = new_cost;
        tree.children{tree.count} = [];
        tree.children{x_nearest_idx}(end+1) = tree.count;
        
        counter = counter + 1;
        
        if enableVisualization && mod(iter, visualizationInterval) == 0
            if m == 2
                plot([x_nearest(1) x_new(1)], [x_nearest(2) x_new(2)], 'b-', 'LineWidth', 0.5);
            else
                plot3([x_nearest(1) x_new(1)], [x_nearest(2) x_new(2)], ...
                      [x_nearest(3) x_new(3)], 'b-', 'LineWidth', 0.5);
            end
            drawnow;
        end
    end
    
    % 6. 周期性Pareto选择新起点
    if counter >= interval
        [newStartIdx, newStart] = ChooseNodePareto(tree, goalPoint, paretoProb, currentStartIdx);
        if ~CheckCollision(currentStart, newStart, obstacles, m)
            currentStart = newStart;
            currentStartIdx = newStartIdx;
        end
        counter = 0;
    end
end

% ========== 结果处理 ==========
if ~success
    fprintf('未能在%d次迭代内找到路径\n', maxIterations);
    path = [];
    metrics = struct('success', false, 'convergenceTime', NaN, ...
                     'pathLength', NaN, 'iterations', maxIterations, ...
                     'nodeCount', tree.count);
    return;
end

% 提取路径
path = ExtractPath(tree, cIdx, startPoint);

% 计算路径长度
pathLength = CalculatePathLength(path);

fprintf('路径规划成功!\n');
fprintf('收敛时间: %.4f秒\n', convergenceTime);
fprintf('路径长度: %.2f\n', pathLength);
fprintf('节点数: %d\n', tree.count);
fprintf('首次解迭代: %d\n', firstSolutionIter);

% 可视化最终路径
if enableVisualization
    if m == 2
        plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
    else
        plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2);
    end
    title(sprintf('Dynamic RRT (Interval=%d) - Time: %.4fs, Length: %.2f', ...
                  interval, convergenceTime, pathLength));
end

% 输出指标
metrics = struct();
metrics.success = true;
metrics.convergenceTime = convergenceTime;
metrics.pathLength = pathLength;
metrics.iterations = firstSolutionIter;
metrics.nodeCount = tree.count;
metrics.interval = interval;

end

% ========== 辅助函数 ==========
function path = ExtractPath(tree, goalIdx, startPoint)
    path = [];
    currentIdx = goalIdx;
    while currentIdx ~= 0
        path = [tree.nodes(currentIdx, :); path];
        currentIdx = tree.parents(currentIdx);
    end
end

function pathLength = CalculatePathLength(path)
    pathLength = 0;
    for i = 1:size(path, 1)-1
        pathLength = pathLength + norm(path(i+1, :) - path(i, :));
    end
end
