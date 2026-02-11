function collision = CheckCollision(x_from, x_to, obstacles, m)
% CheckCollision - 检查从x_from到x_to的路径是否与障碍物碰撞
%
% 输入:
%   x_from    - 起始点 [1×m]
%   x_to      - 终止点 [1×m]
%   obstacles - 障碍物结构体
%   m         - 空间维度
%
% 输出:
%   collision - true表示有碰撞，false表示无碰撞

collision = false;

% 检测步数（根据距离自适应）
distance = norm(x_to - x_from);
num_checks = max(10, ceil(distance / 5));  % 至少检查10个点

% 沿路径插值检查
for i = 0:num_checks
    t = i / num_checks;
    x_check = x_from + t * (x_to - x_from);
    
    % 检查与障碍物的碰撞
    if m == 2
        % 2D: 圆形障碍物
        if isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
            for j = 1:size(obstacles.circles, 1)
                obs_center = obstacles.circles(j, 1:2);
                obs_radius = obstacles.circles(j, 3);
                
                if norm(x_check - obs_center) <= obs_radius
                    collision = true;
                    return;
                end
            end
        end
    else
        % 3D: 球形障碍物
        if isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
            for j = 1:size(obstacles.spheres, 1)
                obs_center = obstacles.spheres(j, 1:3);
                obs_radius = obstacles.spheres(j, 4);
                
                if norm(x_check - obs_center) <= obs_radius
                    collision = true;
                    return;
                end
            end
        end
    end
end

end
