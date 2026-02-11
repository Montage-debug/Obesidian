function obstacles = GenerateObstacles(dimension, bounds, numObstacles, obstacleRadius, startPoint, goalPoint, seed)
% GenerateObstacles - 生成随机分布的障碍物
%
% 论文环境配置 (Table 1):
%   2D: 1500×1500, 225个障碍物, start(400,400), goal(1100,1100)
%   3D: 1500×1500×1500, 400个障碍物, start(1,1,1), goal(1100,1100,1100)
%
% 输入:
%   dimension       - '2D' 或 '3D'
%   bounds          - 空间边界 [xmin xmax ymin ymax] 或 [xmin xmax ymin ymax zmin zmax]
%   numObstacles    - 障碍物数量
%   obstacleRadius  - 障碍物半径
%   startPoint      - 起始点（避免在此处生成障碍物）
%   goalPoint       - 目标点（避免在此处生成障碍物）
%   seed            - 随机种子（如果是NaN则不设置）
%
% 输出:
%   obstacles - 障碍物结构体
%               2D: obstacles.circles [N×3] 每行为 [x, y, radius]
%               3D: obstacles.spheres [N×4] 每行为 [x, y, z, radius]

% 设置随机种子（如果提供）
if ~isnan(seed)
    rng(seed);
end

obstacles = struct();

if strcmp(dimension, '2D')
    % 2D障碍物生成
    obstacles.circles = zeros(numObstacles, 3);
    
    % 安全区域半径（起点和终点周围不生成障碍物）
    safeRadius = 3 * obstacleRadius;
    
    idx = 1;
    maxAttempts = numObstacles * 100;  % 最大尝试次数
    attempts = 0;
    
    while idx <= numObstacles && attempts < maxAttempts
        attempts = attempts + 1;
        
        % 随机生成障碍物中心
        x = bounds(1) + rand * (bounds(2) - bounds(1));
        y = bounds(3) + rand * (bounds(4) - bounds(4));
        center = [x, y];
        
        % 检查是否在边界内（留出半径空间）
        if x < bounds(1) + obstacleRadius || x > bounds(2) - obstacleRadius || ...
           y < bounds(3) + obstacleRadius || y > bounds(4) - obstacleRadius
            continue;
        end
        
        % 检查是否离起点或终点太近
        if norm(center - startPoint) < safeRadius || ...
           norm(center - goalPoint) < safeRadius
            continue;
        end
        
        % 检查是否与已有障碍物重叠
        overlap = false;
        if idx > 1
            for j = 1:idx-1
                existingCenter = obstacles.circles(j, 1:2);
                existingRadius = obstacles.circles(j, 3);
                if norm(center - existingCenter) < (obstacleRadius + existingRadius + 5)
                    overlap = true;
                    break;
                end
            end
        end
        
        if ~overlap
            obstacles.circles(idx, :) = [center, obstacleRadius];
            idx = idx + 1;
        end
    end
    
    % 如果未能生成足够的障碍物，截断
    if idx <= numObstacles
        obstacles.circles = obstacles.circles(1:idx-1, :);
        fprintf('警告: 只生成了%d个障碍物（目标%d个）\n', idx-1, numObstacles);
    end
    
elseif strcmp(dimension, '3D')
    % 3D障碍物生成
    obstacles.spheres = zeros(numObstacles, 4);
    
    % 安全区域半径
    safeRadius = 3 * obstacleRadius;
    
    idx = 1;
    maxAttempts = numObstacles * 100;
    attempts = 0;
    
    while idx <= numObstacles && attempts < maxAttempts
        attempts = attempts + 1;
        
        % 随机生成障碍物中心
        x = bounds(1) + rand * (bounds(2) - bounds(1));
        y = bounds(3) + rand * (bounds(4) - bounds(4));
        z = bounds(5) + rand * (bounds(6) - bounds(5));
        center = [x, y, z];
        
        % 检查是否在边界内
        if x < bounds(1) + obstacleRadius || x > bounds(2) - obstacleRadius || ...
           y < bounds(3) + obstacleRadius || y > bounds(4) - obstacleRadius || ...
           z < bounds(5) + obstacleRadius || z > bounds(6) - obstacleRadius
            continue;
        end
        
        % 检查是否离起点或终点太近
        if norm(center - startPoint) < safeRadius || ...
           norm(center - goalPoint) < safeRadius
            continue;
        end
        
        % 检查是否与已有障碍物重叠
        overlap = false;
        if idx > 1
            for j = 1:idx-1
                existingCenter = obstacles.spheres(j, 1:3);
                existingRadius = obstacles.spheres(j, 4);
                if norm(center - existingCenter) < (obstacleRadius + existingRadius + 5)
                    overlap = true;
                    break;
                end
            end
        end
        
        if ~overlap
            obstacles.spheres(idx, :) = [center, obstacleRadius];
            idx = idx + 1;
        end
    end
    
    % 如果未能生成足够的障碍物，截断
    if idx <= numObstacles
        obstacles.spheres = obstacles.spheres(1:idx-1, :);
        fprintf('警告: 只生成了%d个障碍物（目标%d个）\n', idx-1, numObstacles);
    end
else
    error('不支持的维度: %s', dimension);
end

end
