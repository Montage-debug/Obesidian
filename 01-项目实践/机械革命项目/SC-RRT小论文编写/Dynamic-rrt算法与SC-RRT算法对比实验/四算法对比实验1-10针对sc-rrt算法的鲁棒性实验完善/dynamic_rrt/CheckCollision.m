function collision = CheckCollision(x_from, x_to, obstacles, m)
% CheckCollision - 检查从x_from到x_to的路径是否与障碍物碰撞 (向量化版本)
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
num_checks = max(5, ceil(distance / 10));  % 适当减少检查点数量

direction = x_to - x_from;

if m == 2
    if isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
        obs_centers = obstacles.circles(:, 1:2);  % [N_obs x 2]
        obs_radii = obstacles.circles(:, 3);       % [N_obs x 1]
        
        for i = 0:num_checks
            t = i / num_checks;
            x_check = x_from + t * direction;
            % 向量化距离计算: 一次计算到所有障碍物的距离
            dists = vecnorm(obs_centers - x_check, 2, 2);
            if any(dists <= obs_radii)
                collision = true;
                return;
            end
        end
    end
else
    if isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
        obs_centers = obstacles.spheres(:, 1:3);  % [N_obs x 3]
        obs_radii = obstacles.spheres(:, 4);       % [N_obs x 1]
        
        for i = 0:num_checks
            t = i / num_checks;
            x_check = x_from + t * direction;
            % 向量化距离计算
            dists = vecnorm(obs_centers - x_check, 2, 2);
            if any(dists <= obs_radii)
                collision = true;
                return;
            end
        end
    end
end

end
