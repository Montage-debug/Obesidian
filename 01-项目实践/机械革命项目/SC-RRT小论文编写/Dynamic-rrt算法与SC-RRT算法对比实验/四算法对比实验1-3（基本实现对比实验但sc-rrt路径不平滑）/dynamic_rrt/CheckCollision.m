function collision = CheckCollision(x_from, x_to, obstacles, m)
% CheckCollision - 检查从x_from到x_to的路径是否与障碍物碰撞
%
% 输入:
%   x_from    - 起始点 [1 x m]
%   x_to      - 终止点 [1 x m]
%   obstacles - 障碍物结构体 (.circles或.spheres)
%   m         - 空间维度
%
% 输出:
%   collision - true表示有碰撞, false表示无碰撞

collision = false;

distance = norm(x_to - x_from);
num_checks = max(10, ceil(distance / 5));

for i = 0:num_checks
    t = i / num_checks;
    x_check = x_from + t * (x_to - x_from);
    
    if m == 2
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
