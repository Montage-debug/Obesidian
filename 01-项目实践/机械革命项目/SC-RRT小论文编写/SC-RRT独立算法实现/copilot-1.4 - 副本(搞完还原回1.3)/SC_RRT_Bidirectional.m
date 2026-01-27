function [treeA, treeB, path, success, frame_count, metrics] = SC_RRT_Bidirectional(startPoint, goalPoint, bounds, obstacles, fig_handle, gif_filename, frame_interval, delay_time, varargin)
% SC_RRT_Bidirectional - 基于非对称双向约束的双向RRT算法(DE-BRRT)
%
% 功能特点 代码设计 参数说明
%   1. 非对称双向约束采样：基于动态超椭球约束的渐进式采样
%   2. 动态交汇点转移机制：交汇点作为动态目标点，引导双向收敛
%   3. 融合SC-RRT思想：集成Pareto前沿、自适应PID等高级策略
%
% 输入参数:
%   startPoint     - 起始点 [1×m]
%   goalPoint      - 目标点 [1×m]
%   bounds         - 规划空间边界 [1×2m]
%   obstacles      - 障碍物结构体
%   fig_handle     - 图形句柄
%   gif_filename   - GIF文件名
%   frame_interval - GIF帧间隔
%   delay_time     - GIF帧延迟
%   varargin       - 可选参数:
%     'Mode'                  - 搜索模式('basic'|'pid'|'adaptive') 默认'adaptive'
%     'MaxIterations'         - 最大迭代次数 默认10000
%     'UpdateInterval'        - 交汇点更新间隔 默认50
%     'SmoothingFactor'       - 交汇点平滑因子 默认0.7
%     'EllipsoidBuffer'       - 椭球体缓冲系数 默认1.2
%     'VisualizeDualEllipsoid'- 是否可视化双椭球体 默认true
%     'UseParetoFrontier'     - 是否使用Pareto前沿 默认true
%     'ConnectRadius'         - 连接半径 默认自动
%
% 输出:
%   treeA       - 起始树
%   treeB       - 目标树
%   path        - 规划路径
%   success     - 是否成功
%   frame_count - GIF帧数
%   metrics     - 性能指标结构体

% ========== 参数解析 ==========
p = inputParser;
addParameter(p, 'Mode', 'adaptive', @(x) ismember(x, {'basic', 'pid', 'adaptive'}));
addParameter(p, 'MaxIterations', 10000, @isnumeric);
addParameter(p, 'UpdateInterval', 50, @isnumeric);
addParameter(p, 'SmoothingFactor', 0.7, @isnumeric);
addParameter(p, 'EllipsoidBuffer', 1.2, @isnumeric);
addParameter(p, 'VisualizeDualEllipsoid', true, @islogical);
addParameter(p, 'UseParetoFrontier', true, @islogical);
addParameter(p, 'ConnectRadius', [], @isnumeric);
addParameter(p, 'EnableVisualization', true, @islogical);  % 实时可视化控制
addParameter(p, 'VisualizationInterval', 10, @isnumeric);  % 可视化更新间隔
parse(p, varargin{:});

mode = p.Results.Mode;
maxIterations = p.Results.MaxIterations;
updateInterval = p.Results.UpdateInterval;
smoothingFactor = p.Results.SmoothingFactor;
ellipsoidBuffer = p.Results.EllipsoidBuffer;
visualizeDualEllipsoid = p.Results.VisualizeDualEllipsoid;
useParetoFrontier = p.Results.UseParetoFrontier;
connectRadius = p.Results.ConnectRadius;
enableVisualization = p.Results.EnableVisualization;
visualizationInterval = p.Results.VisualizationInterval;

% ========== 维度相关参数 ==========
m = length(startPoint);

if m == 2
    stepSize = 30;
    goalThreshold = 15;
    if isempty(connectRadius)
        connectRadius = 50;
    end
else
    stepSize = 4;
    goalThreshold = 2;
    if isempty(connectRadius)
        connectRadius = 5;
    end
end

% ========== 算法初始化 ==========
fprintf('\n========== SC-RRT双向算法执行 (%dD) ==========\n', m);
fprintf('算法特点：非对称双向约束采样 + 动态交汇点转移\n');
fprintf('自适应模式: %s\n', upper(mode));
fprintf('交汇点更新间隔: %d\n', updateInterval);
fprintf('平滑因子: %.2f\n', smoothingFactor);
fprintf('椭球体缓冲: %.2f\n', ellipsoidBuffer);
if useParetoFrontier
    fprintf('Pareto前沿: 启用\n');
else
    fprintf('Pareto前沿: 禁用\n');
end
if visualizeDualEllipsoid
    fprintf('双椭球体可视化: 启用\n');
else
    fprintf('双椭球体可视化: 禁用\n');
end
fprintf('==========================================\n\n');

% 记录开始时间
tic;

% 初始化树结构
initial_capacity = min(1000, ceil(maxIterations / 10));
treeA = zeros(initial_capacity, m + 4);
treeB = zeros(initial_capacity, m + 4);

treeA(1, 1:m) = startPoint;
treeA(1, m+1:m+4) = [0, 0, inf, 0];
sizeA = 1;

treeB(1, 1:m) = goalPoint;
treeB(1, m+1:m+4) = [0, 0, inf, 0];
sizeB = 1;

% 算法状态
path = [];
success = false;
iterCount = 0;
swapFlag = false;

% 交汇点初始化
meetPoint = (startPoint + goalPoint) / 2;
meetPoint_old = meetPoint;

% 双椭球体参数
c_min_A = norm(meetPoint - startPoint);
c_min_B = norm(goalPoint - meetPoint);
c_best_A = inf;
c_best_B = inf;

% PID控制器参数（为两个树分别设置）
prevErrorA = 0;
integralErrorA = 0;
errorHistoryA = [];

prevErrorB = 0;
integralErrorB = 0;
errorHistoryB = [];

% 性能指标
totalSamples = 0;
validSamplesA = 0;
validSamplesB = 0;
validNodeCountA = 0;
validNodeCountB = 0;
pathLengthHistoryA = [];
pathLengthHistoryB = [];
L_best_shared = inf;

% 可视化句柄
ellipsoidHandleA = [];
ellipsoidHandleB = [];
meetPointHandle = [];

% ========== 可视化初始化 ==========
figure(fig_handle);
hold on;

% 固定坐标轴范围，防止自动缩放
if m == 2
    axis([bounds(1) bounds(2) bounds(3) bounds(4)]);
    axis equal;
else
    axis([bounds(1) bounds(2) bounds(3) bounds(4) bounds(5) bounds(6)]);
    axis equal;
end

% GIF初始化
gif_frame_count = 0;
if ~isempty(gif_filename)
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    imwrite(imind, cm, gif_filename, 'gif', 'DelayTime', delay_time);
    gif_frame_count = gif_frame_count + 1;
end

% ========== 主循环 ==========
while iterCount < maxIterations && ~success
    iterCount = iterCount + 1;
    
    % ========== 1. 定期更新交汇点和双椭球体 ==========
    if mod(iterCount, updateInterval) == 0 && sizeA > 1 && sizeB > 1
        % 使用势场法计算新的交汇点
        [meetPoint_new, centroidA, centroidB] = calculatePotentialMeetPoint(...
            treeA(1:sizeA, :), treeB(1:sizeB, :), startPoint, goalPoint, m);
        
        % 平滑更新，避免振荡
        meetPoint = smoothingFactor * meetPoint_old + ...
                    (1 - smoothingFactor) * meetPoint_new;
        meetPoint_old = meetPoint;
        
        % 算法的核心：更新双椭球体约束参数
        [c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(...
            treeA(1:sizeA, :), treeB(1:sizeB, :), ...
            startPoint, goalPoint, meetPoint, m, ellipsoidBuffer);
        
        % 计算采样效率
        validRateA = validSamplesA / max(1, totalSamples / 2);
        validRateB = validSamplesB / max(1, totalSamples / 2);
        totalValidRate = (validSamplesA + validSamplesB) / max(1, totalSamples);
        
        % 精简的进度输出（每200次迭代显示一次）
        if mod(iterCount, 200) == 0
            fprintf('  迭代%d: 交汇点[%.0f,%.0f], c_A=%.1f, c_B=%.1f, 效率=%.0f%%\n', ...
                iterCount, meetPoint(1), meetPoint(2), c_best_A, c_best_B, totalValidRate*100);
        end
        
        % ========== 可视化双椭球体 ==========
        if visualizeDualEllipsoid && enableVisualization
            try
                % 安全删除旧的超椭球句柄（改进版 - 使用 isgraphics）
                % 删除椭球体A
                if ~isempty(ellipsoidHandleA)
                    try
                        if iscell(ellipsoidHandleA)
                            for idx = 1:length(ellipsoidHandleA)
                                if isgraphics(ellipsoidHandleA{idx})
                                    delete(ellipsoidHandleA{idx});
                                end
                            end
                        elseif isgraphics(ellipsoidHandleA)
                            delete(ellipsoidHandleA);
                        end
                    catch
                        % 忽略删除错误
                    end
                    ellipsoidHandleA = [];
                end
                
                % 删除椭球体B
                if ~isempty(ellipsoidHandleB)
                    try
                        if iscell(ellipsoidHandleB)
                            for idx = 1:length(ellipsoidHandleB)
                                if isgraphics(ellipsoidHandleB{idx})
                                    delete(ellipsoidHandleB{idx});
                                end
                            end
                        elseif isgraphics(ellipsoidHandleB)
                            delete(ellipsoidHandleB);
                        end
                    catch
                        % 忽略删除错误
                    end
                    ellipsoidHandleB = [];
                end
                
                % 删除交汇点标记
                if ~isempty(meetPointHandle)
                    try
                        if isgraphics(meetPointHandle)
                            delete(meetPointHandle);
                        end
                    catch
                        % 忽略删除错误
                    end
                    meetPointHandle = [];
                end
                
                % 绘制超椭球A（绿色，起点→交汇点）
                if isfinite(c_best_A) && isfinite(c_min_A) && c_best_A > c_min_A * 1.02
                    ellipsoidHandleA = plotEllipsoid(startPoint, meetPoint, c_best_A, m, [0.2 0.8 0.2]);
                end
                
                % 绘制超椭球B（蓝色，交汇点→终点）
                if isfinite(c_best_B) && isfinite(c_min_B) && c_best_B > c_min_B * 1.02
                    ellipsoidHandleB = plotEllipsoid(meetPoint, goalPoint, c_best_B, m, [0.2 0.2 0.8]);
                end
                
                % 标记交汇点
                if m == 2
                    meetPointHandle = scatter(meetPoint(1), meetPoint(2), 150, 'y', 'filled', 'd', ...
                        'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Meet Point');
                else
                    meetPointHandle = scatter3(meetPoint(1), meetPoint(2), meetPoint(3), 150, 'y', 'filled', 'd', ...
                        'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Meet Point');
                end
                
                % 强制刷新显示
                drawnow limitrate;
                
            catch ME
                % 椭球体绘制异常处理
                ellipsoidHandleA = [];
                ellipsoidHandleB = [];
                meetPointHandle = [];
            end
        end
    end
    
    % ========== 2. 评估搜索效率（adaptive模式） ==========
    if strcmp(mode, 'adaptive')
        searchEfficiencyA = PerformanceModule('search_efficiency', validNodeCountA, sizeA, pathLengthHistoryA, 100);
        searchEfficiencyB = PerformanceModule('search_efficiency', validNodeCountB, sizeB, pathLengthHistoryB, 100);
    else
        searchEfficiencyA = 0.5;
        searchEfficiencyB = 0.5;
    end
    
    % ========== 3. 为A树生成采样点（基于椭球A内，带目标偏置） ==========
    randomPointA = generateDualEllipsoidSample(...
        bounds, startPoint, meetPoint, c_best_A, c_min_A, m, 0.3, goalThreshold);
    
    totalSamples = totalSamples + 1;
    if isInsideEllipsoid(randomPointA, startPoint, meetPoint, c_best_A)
        validSamplesA = validSamplesA + 1;
    end
    
    % ========== 4. 扩展A树（使用Pareto前沿动态节点） ==========
    if useParetoFrontier && mod(iterCount, 100) == 0 && sizeA > 10
        % 使用Pareto前沿选择动态节点
        try
            [dynamicStartA, ~, ~, ~] = ParetoModule(treeA(1:sizeA, :), meetPoint, 0.1, m);
            [nearestIdxA, nearestPointA] = findNearPoint(treeA(1:sizeA, :), randomPointA);
            
            % 判断是否从动态节点扩展更好
            distFromDynamic = norm(randomPointA - dynamicStartA);
            distFromNearest = norm(randomPointA - nearestPointA);
            if distFromDynamic < distFromNearest
                % 找到dynamicStartA在树中的索引
                dists = sqrt(sum((treeA(1:sizeA, 1:m) - repmat(dynamicStartA, sizeA, 1)).^2, 2));
                [~, nearestIdxA] = min(dists);
                nearestPointA = treeA(nearestIdxA, 1:m);
            end
        catch
            [nearestIdxA, nearestPointA] = findNearPoint(treeA(1:sizeA, :), randomPointA);
        end
    else
        [nearestIdxA, nearestPointA] = findNearPoint(treeA(1:sizeA, :), randomPointA);
    end
    
    newPointA = expandPoint(nearestPointA, randomPointA, stepSize);
    
    if isCollisionFree(nearestPointA, newPointA, obstacles, m)
        % 添加新节点到树A
        newCostA = treeA(nearestIdxA, m+2) + norm(newPointA - nearestPointA);
        sizeA = sizeA + 1;
        
        if sizeA > size(treeA, 1)
            treeA = [treeA; zeros(size(treeA, 1), m + 4)];
        end
        
        % 节点重连策略：向量化搜索半径内更优父节点
        rewireRadius = min(stepSize * 2, 5.0);
        bestParentIdx = nearestIdxA;
        bestCost = newCostA;
        
        % 向量化计算所有节点到newPointA的距离
        nodePositions = treeA(1:sizeA-1, 1:m);
        diffs = nodePositions - newPointA;
        dists = sqrt(sum(diffs.^2, 2));
        
        % 找出半径内的候选节点
        candidateIdx = find(dists < rewireRadius);
        
        for k = 1:length(candidateIdx)
            idx = candidateIdx(k);
            potentialCost = treeA(idx, m+2) + dists(idx);
            if potentialCost < bestCost
                if isCollisionFree(nodePositions(idx,:), newPointA, obstacles, m)
                    bestParentIdx = idx;
                    bestCost = potentialCost;
                end
            end
        end
        
        % 使用最优父节点
        treeA(sizeA, 1:m) = newPointA;
        treeA(sizeA, m+1) = bestParentIdx;
        treeA(sizeA, m+2) = bestCost;
        
        % 使用SC-RRT的自适应式代价函数
        try
            [F_hat, errorInfo] = CostModule(treeA(1:sizeA, :), sizeA, meetPoint, m, ...
                'Mode', mode, ...
                'PrevError', prevErrorA, ...
                'IntegralError', integralErrorA, ...
                'BestPathLength', L_best_shared, ...
                'IterCount', iterCount, ...
                'MaxIterations', maxIterations, ...
                'SearchEfficiency', searchEfficiencyA, ...
                'ErrorHistory', errorHistoryA);
            
            treeA(sizeA, m+3) = F_hat;
            prevErrorA = errorInfo.currentError;
            integralErrorA = errorInfo.integralError;
            if strcmp(mode, 'adaptive') || strcmp(mode, 'pid')
                errorHistoryA = [errorHistoryA, errorInfo.currentError];
                if length(errorHistoryA) > 10
                    errorHistoryA = errorHistoryA(end-9:end);
                end
            end
        catch
            treeA(sizeA, m+3) = newCostA + norm(newPointA - meetPoint);
        end
        
        treeA(sizeA, m+4) = 0;
        treeA(nearestIdxA, m+4) = treeA(nearestIdxA, m+4) + 1;
        
        % 绘制树A（可视化）
        if enableVisualization && mod(iterCount, visualizationInterval) == 0
            if m == 2
                line([nearestPointA(1), newPointA(1)], [nearestPointA(2), newPointA(2)], ...
                    'Color', [0.5, 0.9, 0.5], 'LineWidth', 0.5);
                scatter(newPointA(1), newPointA(2), 3, 'g', 'filled');
            else
                line([nearestPointA(1), newPointA(1)], [nearestPointA(2), newPointA(2)], ...
                    [nearestPointA(3), newPointA(3)], 'Color', [0.5, 0.9, 0.5], 'LineWidth', 0.5);
                scatter3(newPointA(1), newPointA(2), newPointA(3), 3, 'g', 'filled');
            end
            drawnow;
        end
        
        % ========== 5. 尝试直接连接B树 ==========
        [nearestIdxB, nearestPointB] = findNearPoint(treeB(1:sizeB, :), newPointA);
        distAB = norm(newPointA - nearestPointB);
        
        if distAB <= connectRadius
            if isCollisionFree(newPointA, nearestPointB, obstacles, m)
                % 连接成功！
                path = buildBidirectionalPath(treeA(1:sizeA, :), sizeA, ...
                                             treeB(1:sizeB, :), nearestIdxB, m);
                success = true;
                L_best_shared = calculatePathLength(path);
                
                fprintf('✅ 迭代%d: 找到路径! 连接距离=%.2f, 路径长度=%.2f\n', ...
                    iterCount, distAB, L_best_shared);
                break;
            end
        end
        
        % ========== 6. Connect步骤：从B树向newPointA扩展 ==========
        extendPointB = nearestPointB;
        extendIdxB = nearestIdxB;
        connectAttempts = 0;
        maxConnectAttempts = ceil(distAB / stepSize) + 5;
        
        while norm(newPointA - extendPointB) > stepSize && connectAttempts < maxConnectAttempts
            connectAttempts = connectAttempts + 1;
            newStepB = expandPoint(extendPointB, newPointA, stepSize);
            
            if ~isCollisionFree(extendPointB, newStepB, obstacles, m)
                break;
            end
            
            % 添加节点到树B（包含重连策略）
            newCostB = treeB(extendIdxB, m+2) + norm(newStepB - extendPointB);
            sizeB = sizeB + 1;
            
            if sizeB > size(treeB, 1)
                treeB = [treeB; zeros(size(treeB, 1), m + 4)];
            end
            
            % 节点重连策略：向量化搜索半径内更优父节点
            rewireRadius = min(stepSize * 2, 5.0);
            bestParentIdxB = extendIdxB;
            bestCostB = newCostB;
            
            % 向量化计算所有节点到newStepB的距离
            nodePositionsB = treeB(1:sizeB-1, 1:m);
            diffsB = nodePositionsB - newStepB;
            distsB = sqrt(sum(diffsB.^2, 2));
            
            % 找出半径内的候选节点
            candidateIdxB = find(distsB < rewireRadius);
            
            for k = 1:length(candidateIdxB)
                idx = candidateIdxB(k);
                potentialCost = treeB(idx, m+2) + distsB(idx);
                if potentialCost < bestCostB
                    if isCollisionFree(nodePositionsB(idx,:), newStepB, obstacles, m)
                        bestParentIdxB = idx;
                        bestCostB = potentialCost;
                    end
                end
            end
            
            % 使用最优父节点
            treeB(sizeB, 1:m) = newStepB;
            treeB(sizeB, m+1) = bestParentIdxB;
            treeB(sizeB, m+2) = bestCostB;
            
            % 使用自适应代价
            try
                [F_hat_B, errorInfoB] = CostModule(treeB(1:sizeB, :), sizeB, meetPoint, m, ...
                    'Mode', mode, ...
                    'PrevError', prevErrorB, ...
                    'IntegralError', integralErrorB, ...
                    'BestPathLength', L_best_shared, ...
                    'IterCount', iterCount, ...
                    'MaxIterations', maxIterations, ...
                    'SearchEfficiency', searchEfficiencyB, ...
                    'ErrorHistory', errorHistoryB);
                
                treeB(sizeB, m+3) = F_hat_B;
                prevErrorB = errorInfoB.currentError;
                integralErrorB = errorInfoB.integralError;
            catch
                treeB(sizeB, m+3) = bestCostB + norm(newStepB - meetPoint);
            end
            
            treeB(sizeB, m+4) = 0;
            treeB(extendIdxB, m+4) = treeB(extendIdxB, m+4) + 1;
            
            % 绘制树B（可视化）
            if enableVisualization && mod(iterCount, visualizationInterval) == 0
                if m == 2
                    line([extendPointB(1), newStepB(1)], [extendPointB(2), newStepB(2)], ...
                        'Color', [0.5, 0.5, 0.9], 'LineWidth', 0.5);
                    scatter(newStepB(1), newStepB(2), 3, 'b', 'filled');
                else
                    line([extendPointB(1), newStepB(1)], [extendPointB(2), newStepB(2)], ...
                        [extendPointB(3), newStepB(3)], 'Color', [0.5, 0.5, 0.9], 'LineWidth', 0.5);
                    scatter3(newStepB(1), newStepB(2), newStepB(3), 3, 'b', 'filled');
                end
                drawnow;
            end
            
            extendPointB = newStepB;
            extendIdxB = sizeB;
            
            % 检查是否到达
            if norm(newPointA - extendPointB) <= stepSize
                path = buildBidirectionalPath(treeA(1:sizeA, :), sizeA, ...
                                             treeB(1:sizeB, :), sizeB, m);
                success = true;
                L_best_shared = calculatePathLength(path);
                
                fprintf('✅ 迭代%d: Connect成功! 路径长度=%.2f\n', iterCount, L_best_shared);
                break;
            end
        end
        
        if success
            break;
        end
    end
    
    % ========== 7. 交换树角色 ==========
    swapFlag = ~swapFlag;
    if swapFlag
        [treeA, treeB] = deal(treeB, treeA);
        [sizeA, sizeB] = deal(sizeB, sizeA);
        [c_best_A, c_best_B] = deal(c_best_B, c_best_A);
        [c_min_A, c_min_B] = deal(c_min_B, c_min_A);
        [prevErrorA, prevErrorB] = deal(prevErrorB, prevErrorA);
        [integralErrorA, integralErrorB] = deal(integralErrorB, integralErrorA);
        [errorHistoryA, errorHistoryB] = deal(errorHistoryB, errorHistoryA);
        [validSamplesA, validSamplesB] = deal(validSamplesB, validSamplesA);
    end
    
    % ========== 8. 保存GIF帧 ==========
    if ~isempty(gif_filename) && mod(iterCount, frame_interval) == 0
        drawnow;
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        gif_frame_count = gif_frame_count + 1;
    end
    
    % ========== 9. 进度显示 ==========
    if mod(iterCount, 500) == 0
        validRate = (validSamplesA + validSamplesB) / max(1, totalSamples);
        elapsedTime = toc;
        estimatedTotal = elapsedTime / iterCount * maxIterations;
        remainingTime = estimatedTotal - elapsedTime;
        
        fprintf('\n========== 进度报告 [迭代: %d/%d, %.1f%%] ==========\n', ...
            iterCount, maxIterations, (iterCount/maxIterations)*100);
        fprintf('⏱️  已用时间: %.2f秒, 预计剩余: %.2f秒\n', elapsedTime, remainingTime);
        fprintf('🌲 树A节点: %d, 树B节点: %d, 总计: %d\n', sizeA, sizeB, sizeA+sizeB);
        fprintf('📊 有效采样率: %.1f%% (%d/%d)\n', validRate * 100, validSamplesA+validSamplesB, totalSamples);
        
        if ~isinf(L_best_shared)
            fprintf('✨ 当前最优路径长度: %.2f\n', L_best_shared);
        end
        
        fprintf('=============================================\n\n');
    end
end

% 记录结束时间
computeTime = toc;

% ========== 最终可视化 ==========
if enableVisualization
    if m == 2
        scatter(startPoint(1), startPoint(2), 150, 'g', 'filled', 'pentagram', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Start');
        scatter(goalPoint(1), goalPoint(2), 150, 'r', 'filled', 'pentagram', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Goal');
        
        if success && ~isempty(path)
            plot(path(:,1), path(:,2), 'r-', 'LineWidth', 3, 'DisplayName', 'Final Path');
        end
    else
        scatter3(startPoint(1), startPoint(2), startPoint(3), 150, 'g', 'filled', 'pentagram');
        scatter3(goalPoint(1), goalPoint(2), goalPoint(3), 150, 'r', 'filled', 'pentagram');
        
        if success && ~isempty(path)
            plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 3);
        end
    end
    drawnow;
end

legend('Location', 'best');

% 保存最后一帧
if ~isempty(gif_filename)
    drawnow;
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
    gif_frame_count = gif_frame_count + 1;
end

frame_count = gif_frame_count;

% 裁剪树
treeA = treeA(1:sizeA, :);
treeB = treeB(1:sizeB, :);

% ========== 计算性能指标 ==========
metrics = struct();
metrics.iterCount = iterCount;
metrics.computeTime = computeTime;
metrics.validSamplingRate = (validSamplesA + validSamplesB) / totalSamples;
metrics.treeASize = sizeA;
metrics.treeBSize = sizeB;
metrics.totalNodes = sizeA + sizeB;

if success
    metrics.pathLength = L_best_shared;
    metrics.pathNodes = size(path, 1);
    metrics.pathSmoothness = calculatePathSmoothness(path);
else
    metrics.pathLength = inf;
    metrics.pathNodes = 0;
    metrics.pathSmoothness = inf;
end

% ========== 执行摘要 ==========
fprintf('\n========== SC-RRT双向算法执行结果 ==========\n');
if success
    fprintf('状态: ✅ 成功\n');
    fprintf('总迭代次数: %d\n', iterCount);
    fprintf('计算时间: %.4f 秒\n', computeTime);
    fprintf('树A节点数: %d\n', sizeA);
    fprintf('树B节点数: %d\n', sizeB);
    fprintf('总节点数: %d\n', sizeA + sizeB);
    fprintf('路径长度: %.2f\n', metrics.pathLength);
    fprintf('路径节点数: %d\n', metrics.pathNodes);
    fprintf('路径平滑度: %.4f\n', metrics.pathSmoothness);
    fprintf('有效采样率: %.2f%%\n', metrics.validSamplingRate * 100);
else
    fprintf('状态: ❌ 失败\n');
    fprintf('总迭代次数: %d\n', iterCount);
    fprintf('计算时间: %.4f 秒\n', computeTime);
    fprintf('树A节点数: %d\n', sizeA);
    fprintf('树B节点数: %d\n', sizeB);
end
fprintf('=====================================\n\n');

end

%% ========== 辅助函数 ==========

function randomPoint = generateDualEllipsoidSample(bounds, focus1, focus2, c_best, c_min, m, goalBias, biasRadius)
% 双椭球体约束采样

if rand < goalBias
    % 向focus2偏置采样
    randomPoint = sampleGoalBiased(focus2, biasRadius, bounds);
elseif isfinite(c_best) && c_best > c_min && rand < 0.5
    % 椭球体内采样
    try
        randomPoint = SampleEllipsoid(focus1, focus2, c_best, bounds, struct(), 1, m);
        if size(randomPoint, 1) == m && size(randomPoint, 2) == 1
            randomPoint = randomPoint';
        end
        
        % 验证
        if ~isInsideEllipsoid(randomPoint, focus1, focus2, c_best)
            randomPoint = samplePoint(bounds, focus2);
        end
    catch
        randomPoint = samplePoint(bounds, focus2);
    end
else
    % 均匀采样
    randomPoint = samplePoint(bounds, focus2);
end

end

function isInside = isInsideEllipsoid(point, focus1, focus2, c_best)
% 判断点是否在超椭球内

dist = norm(point - focus1) + norm(point - focus2);
isInside = dist <= c_best * 1.002;
end

function path = buildBidirectionalPath(treeA, endIdxA, treeB, endIdxB, m)
% 构建双向路径（增强鲁棒性，防止索引越界）

% 参数验证
if endIdxA < 1 || endIdxA > size(treeA, 1)
    error('endIdxA索引越界: %d (树A大小: %d)', endIdxA, size(treeA, 1));
end
if endIdxB < 1 || endIdxB > size(treeB, 1)
    error('endIdxB索引越界: %d (树B大小: %d)', endIdxB, size(treeB, 1));
end

% 构建路径A（从起点到连接点）
pathA = [];
currentIdx = endIdxA;
visitedA = false(size(treeA, 1), 1);  % 防止循环

while currentIdx > 0
    % 检查是否访问过（防止循环）
    if visitedA(currentIdx)
        warning('检测到树A中的循环引用，节点%d', currentIdx);
        break;
    end
    visitedA(currentIdx) = true;
    
    pathA = [treeA(currentIdx, 1:m); pathA];
    parentIdx = treeA(currentIdx, m+1);
    
    % 验证父节点索引
    if parentIdx > 0 && (parentIdx < 1 || parentIdx > size(treeA, 1))
        warning('树A父节点索引异常: %d (当前节点: %d)', parentIdx, currentIdx);
        break;
    end
    
    currentIdx = parentIdx;
end

% 构建路径B（从连接点到目标点）
pathB = [];
currentIdx = endIdxB;
visitedB = false(size(treeB, 1), 1);  % 防止循环

while currentIdx > 0
    % 检查是否访问过（防止循环）
    if visitedB(currentIdx)
        warning('检测到树B中的循环引用，节点%d', currentIdx);
        break;
    end
    visitedB(currentIdx) = true;
    
    pathB = [treeB(currentIdx, 1:m); pathB];
    parentIdx = treeB(currentIdx, m+1);
    
    % 验证父节点索引
    if parentIdx > 0 && (parentIdx < 1 || parentIdx > size(treeB, 1))
        warning('树B父节点索引异常: %d (当前节点: %d)', parentIdx, currentIdx);
        break;
    end
    
    currentIdx = parentIdx;
end

% 合并路径（翻转pathB因为是从终点回溯）
pathB = flipud(pathB);

% 去除重复的连接点（如果存在）
if ~isempty(pathA) && ~isempty(pathB)
    if norm(pathA(end, :) - pathB(1, :)) < 1e-6
        path = [pathA; pathB(2:end, :)];
    else
        path = [pathA; pathB];
    end
else
    path = [pathA; pathB];
end

% 最终验证：路径不能为空
if isempty(path)
    error('构建的路径为空！检查树结构是否正确');
end

end

function h = plotEllipsoid(focus1, focus2, cBest, m, color)
% 绘制超椭球（增强稳定性和数值鲁棒性）
% 输入:
%   focus1, focus2 - 椭球体的两个焦点 [1×m]
%   cBest - 椭球约束参数（焦点距离之和）
%   m - 维度（2或3）
%   color - 颜色 [1×3]
% 输出:
%   h - 图形句柄

% 容差阈值
epsilon = 1e-6;

% 确保输入为行向量
if size(focus1, 1) > 1
    focus1 = focus1';
end
if size(focus2, 1) > 1
    focus2 = focus2';
end

% 计算焦距（两焦点之间的距离）
cMin = norm(focus2(1:m) - focus1(1:m));

% 边界检查：cBest必须大于等于cMin（椭球定义）
if ~isfinite(cBest) || ~isfinite(cMin) || cBest < cMin * (1 + epsilon)
    h = [];
    return;
end

% 计算椭球参数
% a: 半长轴（从中心到椭球边界沿主轴方向的距离）
% c: 半焦距（从中心到焦点的距离）  
% b: 半短轴
a = cBest / 2;           % 半长轴 = cBest/2
c_focal = cMin / 2;      % 半焦距 = cMin/2
b_squared = a^2 - c_focal^2;  % b^2 = a^2 - c^2（椭球几何关系）

% 数值稳定性检查
if b_squared < epsilon
    h = [];
    return;
end

b = sqrt(b_squared);

% 椭球中心（两焦点中点）
center = (focus1(1:m) + focus2(1:m)) / 2;

% 主轴方向向量（从focus1指向focus2，归一化）
eVec = (focus2(1:m) - focus1(1:m)) / cMin;

try
    if m == 2
        % ========== 2D椭圆绘制 ==========
        theta = atan2(eVec(2), eVec(1));
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        % 参数方程生成椭圆
        t = linspace(0, 2*pi, 100);
        ellipse = [a*cos(t); b*sin(t)];
        rotated_ellipse = R * ellipse + center';
        
        h = plot(rotated_ellipse(1,:), rotated_ellipse(2,:), '--', ...
            'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        
    elseif m == 3
        % ========== 3D椭球绘制（改进旋转矩阵计算） ==========
        
        % 目标：将标准椭球（长轴沿x轴）旋转到eVec方向
        % 使用两步旋转法构造旋转矩阵
        
        % 标准基向量
        x_axis = [1; 0; 0];
        
        % 计算旋转轴和旋转角
        rotation_axis = cross(x_axis, eVec);
        rotation_axis_norm = norm(rotation_axis);
        
        if rotation_axis_norm < epsilon
            % eVec与x轴平行或反平行
            if dot(x_axis, eVec) > 0
                % 同向，无需旋转
                R = eye(3);
            else
                % 反向，绕y轴旋转180度
                R = [-1 0 0; 0 1 0; 0 0 -1];
            end
        else
            % 一般情况：使用Rodrigues旋转公式
            k = rotation_axis / rotation_axis_norm;  % 归一化旋转轴
            
            % 计算旋转角度（数值稳定版本）
            cos_theta = dot(x_axis, eVec);
            cos_theta = max(-1, min(1, cos_theta));  % 钳位到[-1,1]
            theta = acos(cos_theta);
            
            % Rodrigues公式: R = I + sin(θ)K + (1-cos(θ))K^2
            K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];  % 反对称矩阵
            R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
        end
        
        % 验证旋转矩阵（调试用）
        % 旋转后x轴应该对齐到eVec方向
        % test_vec = R * x_axis;
        % fprintf('旋转验证: ||R*x - eVec|| = %.6f\n', norm(test_vec - eVec));
        
        % 生成标准椭球网格点（中心在原点，长轴沿x轴）
        [phi, theta_grid] = meshgrid(linspace(0, 2*pi, 20), linspace(0, pi, 15));
        
        % 椭球参数方程: x = a*cos(φ)*sin(θ), y = b*sin(φ)*sin(θ), z = b*cos(θ)
        x_ellipsoid = a * cos(phi) .* sin(theta_grid);
        y_ellipsoid = b * sin(phi) .* sin(theta_grid);
        z_ellipsoid = b * cos(theta_grid);
        
        % 将网格点展平为3×N矩阵
        points = [x_ellipsoid(:)'; y_ellipsoid(:)'; z_ellipsoid(:)'];
        
        % 应用旋转和平移
        rotated_points = R * points + center';
        
        % 重塑为网格形式
        x_rot = reshape(rotated_points(1,:), size(x_ellipsoid));
        y_rot = reshape(rotated_points(2,:), size(y_ellipsoid));
        z_rot = reshape(rotated_points(3,:), size(z_ellipsoid));
        
        % 绘制椭球面
        h = surf(x_rot, y_rot, z_rot, ...
            'FaceAlpha', 0.2, ...          % 半透明
            'EdgeColor', color, ...        % 边缘颜色
            'EdgeAlpha', 0.4, ...          % 边缘透明度
            'FaceColor', color, ...        % 面颜色
            'FaceLighting', 'gouraud', ... % 光照模式
            'HandleVisibility', 'off');
        
        % 添加光源（可选，增强3D效果）
        % light('Position', [1 1 1], 'Style', 'infinite');
        
    else
        h = [];
    end
catch ME
    warning('plotEllipsoid绘制异常: %s\n位置: %s', ME.message, ME.stack(1).name);
    h = [];
end

end

function smoothness = calculatePathSmoothness(path)
% 计算路径平滑度

if size(path, 1) < 3
    smoothness = 0;
    return;
end

totalAngle = 0;
for i = 2:size(path,1)-1
    v1 = path(i,:) - path(i-1,:);
    v2 = path(i+1,:) - path(i,:);
    if norm(v1) > 0 && norm(v2) > 0
        cosAngle = dot(v1,v2) / (norm(v1) * norm(v2));
        cosAngle = max(-1, min(1, cosAngle));
        angle = acos(cosAngle);
        totalAngle = totalAngle + angle;
    end
end

smoothness = totalAngle / (size(path,1) - 2);

end