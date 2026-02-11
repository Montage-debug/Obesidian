function printStatistics(success, path, tree, elapsedTime, frame_count)
% PRINTSTATISTICS 打印算法执行统计信息
%
% 输入参数
%   success      - 规划成功标志
%   path         - 路径坐标 [N x dim]
%   tree         - 树结构数据（双树cell数组）
%   elapsedTime  - 执行时间（秒）
%   frame_count  - GIF帧数

    fprintf('\n========== 算法执行统计 ==========\n');
    
    % 规划状态
    if success
        fprintf('状态: 成功找到路径\n');
    else
        fprintf('状态: 未找到路径\n');
    end
    
    % 时间统计
    fprintf('执行时间: %.4f 秒\n', elapsedTime);
    
    % 树统计
    if iscell(tree)
        % 双树
        fprintf('树A节点数: %d\n', size(tree{1}, 1));
        fprintf('树B节点数: %d\n', size(tree{2}, 1));
        fprintf('总节点数: %d\n', size(tree{1}, 1) + size(tree{2}, 1));
    else
        % 单树
        fprintf('树节点数: %d\n', size(tree, 1));
    end
    
    % 路径统计
    if success && ~isempty(path)
        path_length = sum(sqrt(sum(diff(path).^2, 2)));
        fprintf('路径长度: %.2f\n', path_length);
        fprintf('路径节点数: %d\n', size(path, 1));
        
        % 计算路径平滑度（平均转弯角）
        if size(path, 1) >= 3
            angles = zeros(size(path, 1) - 2, 1);
            for i = 2:size(path, 1)-1
                v1 = path(i, :) - path(i-1, :);
                v2 = path(i+1, :) - path(i, :);
                cos_angle = dot(v1, v2) / (norm(v1) * norm(v2));
                cos_angle = max(-1, min(1, cos_angle)); % 夹角范围
                angles(i-1) = acos(cos_angle) * 180 / pi;
            end
            avg_angle = mean(angles);
            fprintf('平滑转弯角: %.2f 度\n', avg_angle);
        end
    end
    
    % 动画统计
    if nargin >= 5 && ~isempty(frame_count)
        fprintf('GIF帧数: %d\n', frame_count);
    end
    
    fprintf('==================================\n\n');
end