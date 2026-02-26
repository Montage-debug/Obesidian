function [best_idx_A, best_idx_B, best_cost] = findBestConnection(treeA, treeB, obstacles, dim, max_check)
% findBestConnection - 寻找两棵树之间的最优连接
%
% 输入:
%   treeA       - 树A [N×(dim+4)]
%   treeB       - 树B [M×(dim+4)]
%   obstacles   - 障碍物列表
%   dim         - 空间维度
%   max_check   - 最多检查的节点数 (默认: 50)
%
% 输出:
%   best_idx_A  - 树A中的最优连接节点索引
%   best_idx_B  - 树B中的最优连接节点索引
%   best_cost   - 最优连接的总代价
%
% 功能:
%   遍历两棵树的前N个节点(按代价排序)，找到碰撞检测通过且总代价最小的连接
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

    if nargin < 5
        max_check = 50;  % 最多检查前50个节点
    end
    
    sizeA = min(size(treeA, 1), max_check);
    sizeB = min(size(treeB, 1), max_check);
    
    best_cost = inf;
    best_idx_A = [];
    best_idx_B = [];
    
    % 按代价排序，只检查代价较小的节点
    [~, sorted_A] = sort(treeA(:, dim+2));
    [~, sorted_B] = sort(treeB(:, dim+2));
    
    for i = 1:sizeA
        idx_A = sorted_A(i);
        node_A = treeA(idx_A, 1:dim);
        cost_A = treeA(idx_A, dim+2);
        
        for j = 1:sizeB
            idx_B = sorted_B(j);
            node_B = treeB(idx_B, 1:dim);
            cost_B = treeB(idx_B, dim+2);
            
            % 连接距离
            connect_dist = norm(node_A - node_B);
            total_cost = cost_A + cost_B + connect_dist;
            
            % 检查是否比当前最优更好
            if total_cost < best_cost
                % 碰撞检测
                if isCollisionFree(node_A, node_B, obstacles, dim)
                    best_cost = total_cost;
                    best_idx_A = idx_A;
                    best_idx_B = idx_B;
                end
            end
        end
    end
end
