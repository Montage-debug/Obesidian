function new_point = steerPoint(nearest, target, step_size)
% steerPoint - Steer函数：从最近点向目标点移动固定步长
%
% 输入:
%   nearest   - 最近节点 [1×m]
%   target    - 目标点 [1×m]
%   step_size - 步长
%
% 输出:
%   new_point - 新节点 [1×m]
%
% 作者: SC-RRT完全优化版
% 日期: 2025-12-12

% 计算方向向量
direction = target - nearest;
distance = norm(direction);

% 如果距离太小,直接返回目标点
if distance < 1e-6
    new_point = target;
    return;
end

% 如果距离大于步长,移动固定步长；否则直接到达目标点
if distance > step_size
    new_point = nearest + (direction / distance) * step_size;
else
    new_point = target;
end

end
