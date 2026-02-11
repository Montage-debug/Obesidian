function x_new = Steer(x_from, x_to, stepSize)
% Steer - 从x_from朝x_to方向扩展stepSize距离
%
% 输入:
%   x_from   - 起始点 [1×m]
%   x_to     - 目标点 [1×m]
%   stepSize - 扩展步长
%
% 输出:
%   x_new - 新节点位置 [1×m]

direction = x_to - x_from;
distance = norm(direction);

if distance <= stepSize
    % 如果距离小于步长，直接返回目标点
    x_new = x_to;
else
    % 沿方向扩展stepSize
    x_new = x_from + (direction / distance) * stepSize;
end

end
