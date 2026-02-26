function [tree, path, success, metrics, fig_ellipsoid] = SC_RRT_with_Ellipsoid_Display(startPoint, goalPoint, bounds, obstacles, varargin)
% SC_RRT_with_Ellipsoid_Display - SC-RRT算法包装函数，支持椭球体动态显示
%
% 功能: 
%   运行SC-RRT双向算法，同时在单独的图形窗口中动态显示两个椭球体
%   椭球体红色表示从起点到交汇点，蓝色表示从交汇点到终点
%
% 输入参数:
%   startPoint  - 起始点 [1×m]
%   goalPoint   - 目标点 [1×m]
%   bounds      - 规划空间边界 [1×2m]
%   obstacles   - 障碍物结构体
%   varargin    - 可选参数 (传递给SC_RRT_Bidirectional)
%
% 输出参数:
%   tree        - {treeA, treeB} 起点树和目标树
%   path        - 规划路径 [N×m]
%   success     - 是否规划成功
%   metrics     - 性能指标
%   fig_ellipsoid - 椭球体显示窗口句柄

% 维度
m = length(startPoint);

% 创建椭球体显示窗口
fig_ellipsoid = figure('Name', 'SC-RRT 椭球体动态显示', ...
    'Position', [950 100 900 800], ...
    'Renderer', 'opengl', ...
    'DoubleBuffer', 'on');

hold on; axis equal; grid on;
if m == 2
    xlim(bounds(1:2));
    ylim(bounds(3:4));
    xlabel('X (mm)', 'FontSize', 12);
    ylabel('Y (mm)', 'FontSize', 12);
    title('SC-RRT 椭球体约束 (绿→红=起点,蓝=终点)', 'FontSize', 13, 'FontWeight', 'bold');
    
    % 绘制障碍物
    for i = 1:size(obstacles.circles, 1)
        circle = obstacles.circles(i, :);
        rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                              2*circle(3), 2*circle(3)], ...
                 'Curvature', [1 1], ...
                 'FaceColor', [0.3 0.3 0.3], ...
                 'EdgeColor', [0.3 0.3 0.3], ...
                 'LineWidth', 0.5);
    end
    
    % 绘制起点和目标点
    plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2.5);
    plot(goalPoint(1), goalPoint(2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2.5);
    legend('起点', '目标点', 'Location', 'best');
    
else
    xlim(bounds(1:2));
    ylim(bounds(3:4));
    zlim(bounds(5:6));
    xlabel('X (mm)', 'FontSize', 12);
    ylabel('Y (mm)', 'FontSize', 12);
    zlabel('Z (mm)', 'FontSize', 12);
    title('SC-RRT 椭球体约束 3D', 'FontSize', 13, 'FontWeight', 'bold');
    
    % 绘制起点和目标点（3D）
    plot3(startPoint(1), startPoint(2), startPoint(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2.5);
    plot3(goalPoint(1), goalPoint(2), goalPoint(3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2.5);
end

drawnow limitrate;

% 调用SC_RRT_Bidirectional并传递椭球体窗口句柄
% 注意：这里假设已经将改进的SC_RRT版本集成到路径中
% 暂时使用基础的双向RRT实现来演示椭球体显示逻辑

fprintf('SC-RRT椭球体显示模块已初始化\n');

% 创建简化的双向RRT演示（用于测试椭球体显示）
[tree, path, success, metrics] = SimpleBidirectionalRRT_with_Ellipsoid(...
    startPoint, goalPoint, bounds, obstacles, fig_ellipsoid, m, varargin{:});

end

%% ========== 简化的双向RRT实现（带椭球体显示） ==========
function [tree, path, success, metrics] = SimpleBidirectionalRRT_with_Ellipsoid(startPoint, goalPoint, bounds, obstacles, fig_ellipsoid, m, varargin)

% 参数设置
p = inputParser;
addParameter(p, 'MaxIterations', 10000, @isnumeric);
addParameter(p, 'Mode', 'pid', @ischar);
addParameter(p, 'UpdateInterval', 50, @isnumeric);
parse(p, varargin{:});

maxIterations = p.Results.MaxIterations;
updateInterval = p.Results.UpdateInterval;

if m == 2
    stepSize = 30;
    goalThreshold = 20;
    connectRadius = 50;
else
    stepSize = 4;
    goalThreshold = 2;
    connectRadius = 5;
end

% 初始化树
treeA_nodes = startPoint;
treeA_parents = [0];
treeA_costs = [0];

treeB_nodes = goalPoint;
treeB_parents = [0];
treeB_costs = [0];

path = [];
success = false;
iterCount = 0;

% 椭球体显示句柄
ellipsoidHandleA = [];
ellipsoidHandleB = [];
meetPointHandle = [];

meetPoint = (startPoint + goalPoint) / 2;

% 主循环
while iterCount < maxIterations && ~success
    iterCount = iterCount + 1;
    
    % 定期更新椭球体显示
    if mod(iterCount, updateInterval) == 0
        % 删除旧的椭球体
        figure(fig_ellipsoid);
        if ~isempty(ellipsoidHandleA) && isgraphics(ellipsoidHandleA)
            delete(ellipsoidHandleA);
        end
        if ~isempty(ellipsoidHandleB) && isgraphics(ellipsoidHandleB)
            delete(ellipsoidHandleB);
        end
        if ~isempty(meetPointHandle) && isgraphics(meetPointHandle)
            delete(meetPointHandle);
        end
        
        % 计算双椭球体参数
        treeA_struct = [treeA_nodes, ones(length(treeA_costs),1), treeA_costs', zeros(length(treeA_costs),1)];
        treeB_struct = [treeB_nodes, ones(length(treeB_costs),1), treeB_costs', zeros(length(treeB_costs),1)];
        
        [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(...
            treeA_struct, treeB_struct, startPoint, goalPoint, meetPoint, m, 1.2);
        
        % 绘制椭球体A（绿色→红色）
        if isfinite(c_best_A) && isfinite(c_min_A) && c_best_A > c_min_A * 1.02
            ellipsoidHandleA = plotEllipsoid(startPoint, meetPoint, c_best_A, m, [0.8 0.3 0.3]);
        end
        
        % 绘制椭球体B（蓝色）
        if isfinite(c_best_B) && isfinite(c_min_B) && c_best_B > c_min_B * 1.02
            ellipsoidHandleB = plotEllipsoid(meetPoint, goalPoint, c_best_B, m, [0.3 0.3 0.8]);
        end
        
        % 标记交汇点
        if m == 2
            meetPointHandle = scatter(meetPoint(1), meetPoint(2), 200, 'y', 'filled', 'd', ...
                'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', '交汇点');
        else
            meetPointHandle = scatter3(meetPoint(1), meetPoint(2), meetPoint(3), 200, 'y', 'filled', 'd', ...
                'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', '交汇点');
        end
        
        % 更新交汇点位置
        meetPoint_new = (mean(treeA_nodes, 1) + mean(treeB_nodes, 1)) / 2;
        meetPoint = 0.8 * meetPoint + 0.2 * meetPoint_new;
        
        drawnow limitrate;
    end
    
    % ========== 树A的扩展 ==========
    % 随机采样或目标偏置
    if rand < 0.1
        randPoint = goalPoint;
    else
        randPoint = bounds(1:2:end)' + rand(m, 1) .* (bounds(2:2:end)' - bounds(1:2:end)');
    end
    
    % 在树A中找最近的节点
    distA = sqrt(sum((treeA_nodes - repmat(randPoint', size(treeA_nodes, 1), 1)).^2, 2));
    [~, nearIdx] = min(distA);
    nearNode = treeA_nodes(nearIdx, :);
    
    % 向随机点扩展
    direction = randPoint - nearNode;
    direction = direction / norm(direction);
    newNode = nearNode + stepSize * direction;
    
    % 碰撞检测
    if isCollisionFreeSimple(nearNode, newNode, obstacles, m)
        treeA_nodes = [treeA_nodes; newNode];
        treeA_parents = [treeA_parents; nearIdx];
        treeA_costs = [treeA_costs; treeA_costs(nearIdx) + norm(newNode - nearNode)];
        
        % 检测连接
        distToB = sqrt(sum((treeB_nodes - repmat(newNode, size(treeB_nodes, 1), 1)).^2, 2));
        if min(distToB) < connectRadius
            success = true;
            break;
        end
    end
    
    % ========== 树B的扩展 ==========
    if rand < 0.1
        randPoint = startPoint;
    else
        randPoint = bounds(1:2:end)' + rand(m, 1) .* (bounds(2:2:end)' - bounds(1:2:end)');
    end
    
    distB = sqrt(sum((treeB_nodes - repmat(randPoint', size(treeB_nodes, 1), 1)).^2, 2));
    [~, nearIdx] = min(distB);
    nearNode = treeB_nodes(nearIdx, :);
    
    direction = randPoint - nearNode;
    direction = direction / norm(direction);
    newNode = nearNode + stepSize * direction;
    
    if isCollisionFreeSimple(nearNode, newNode, obstacles, m)
        treeB_nodes = [treeB_nodes; newNode];
        treeB_parents = [treeB_parents; nearIdx];
        treeB_costs = [treeB_costs; treeB_costs(nearIdx) + norm(newNode - nearNode)];
        
        distToA = sqrt(sum((treeA_nodes - repmat(newNode, size(treeA_nodes, 1), 1)).^2, 2));
        if min(distToA) < connectRadius
            success = true;
            break;
        end
    end
    
    if mod(iterCount, 1000) == 0
        fprintf('  迭代 %d: 树A %d 节点, 树B %d 节点\n', iterCount, size(treeA_nodes, 1), size(treeB_nodes, 1));
    end
end

% 输出结果
tree = {treeA_nodes, treeA_parents, treeA_costs; treeB_nodes, treeB_parents, treeB_costs};
path = [];
metrics = struct();
metrics.iterations = iterCount;
metrics.nodeCount = size(treeA_nodes, 1) + size(treeB_nodes, 1);
metrics.pathLength = 0;

fprintf('SC-RRT（带椭球体显示）完成: %d 次迭代, 状态=%d\n', iterCount, success);

end

%% ========== 简化碰撞检测 ==========
function isFree = isCollisionFreeSimple(p1, p2, obstacles, m)

isFree = true;

% 采样中间点
nSamples = 5;
for i = 1:nSamples
    p = p1 + (i/nSamples) * (p2 - p1);
    
    % 检测是否在任何障碍物内
    for j = 1:size(obstacles.circles, 1)
        obs = obstacles.circles(j, :);
        center = obs(1:m);
        radius = obs(3);
        
        dist = norm(p(1:m) - center);
        if dist < radius
            isFree = false;
            return;
        end
    end
end

end
