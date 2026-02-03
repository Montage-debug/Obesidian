function optimizedPath = rewirePathNodes(rawPath, obstacles, m, maxIterations)
% rewirePathNodes - 路径节点重连优化（改进版：保守策略）
%
% 功能：
%   通过尝试直连更远的节点来消除路径中的冗余节点，在保证无碰撞的前提下
%   尽可能缩短路径长度并减少转折点
%
% 输入：
%   rawPath       - 原始路径 [N×m]
%   obstacles     - 障碍物结构体
%   m             - 空间维度
%   maxIterations - 最大优化迭代次数（默认2，降低以保留更多节点）
%
% 输出：
%   optimizedPath - 优化后的路径 [M×m]（M <= N）
%
% 改进：
%   - 采用更保守的重连策略，保留更多中间节点
%   - 限制单次跳跃的最大距离
%   - 为后续平滑处理预留安全空间

if nargin < 4
    maxIterations = 2;  % 降低迭代次数，保留更多节点
end

% 输入验证
if size(rawPath, 1) < 2
    optimizedPath = rawPath;
    return;
end

% 初始路径
currentPath = rawPath;

% 计算路径总长度，用于限制单次跳跃
totalLength = 0;
for i = 1:size(rawPath, 1) - 1
    totalLength = totalLength + norm(rawPath(i+1, :) - rawPath(i, :));
end
avgSegmentLength = totalLength / (size(rawPath, 1) - 1);
maxJumpDistance = avgSegmentLength * 5;  % 限制单次跳跃不超过5倍平均段长

% 迭代优化
for iter = 1:maxIterations
    previousPathSize = size(currentPath, 1);
    
    % 执行一轮重连优化
    optimizedPath = [];
    i = 1;
    
    while i <= size(currentPath, 1)
        % 当前节点加入优化路径
        optimizedPath = [optimizedPath; currentPath(i, :)];
        
        % 如果已到达终点，退出
        if i == size(currentPath, 1)
            break;
        end
        
        % 尝试跳过尽可能多的中间节点（带距离限制）
        bestNextIdx = i + 1;  % 默认连接下一个节点
        
        % 从最远的节点开始尝试，但限制最大跳跃距离
        for j = size(currentPath, 1):-1:(i+2)
            distToJ = norm(currentPath(j, :) - currentPath(i, :));
            
            % 限制单次跳跃距离，避免路径太稀疏
            if distToJ > maxJumpDistance
                continue;
            end
            
            if isCollisionFree(currentPath(i, :), currentPath(j, :), obstacles, m)
                % 可以直连到节点j，跳过中间节点
                bestNextIdx = j;
                break;
            end
        end
        
        % 移动到下一个节点
        i = bestNextIdx;
    end
    
    % 更新当前路径
    currentPath = optimizedPath;
    
    % 如果路径长度不再减少，提前退出
    if size(currentPath, 1) == previousPathSize
        break;
    end
end

% 输出优化结果
fprintf('  路径重连优化: 原始节点数=%d, 优化后节点数=%d, 减少%.1f%%\n', ...
    size(rawPath, 1), size(optimizedPath, 1), ...
    (1 - size(optimizedPath, 1) / size(rawPath, 1)) * 100);

end
