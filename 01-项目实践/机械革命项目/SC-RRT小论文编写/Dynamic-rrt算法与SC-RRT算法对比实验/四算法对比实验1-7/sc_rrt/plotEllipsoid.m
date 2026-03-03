function h = plotEllipsoid(focus1, focus2, cBest, m, color)
% plotEllipsoid - 绘制超椭球（增强稳定性和数值鲁棒性）
%
% 输入:
%   focus1, focus2 - 椭球体的两个焦点 [1×m]
%   cBest - 椭球约束参数（焦点距离之和）
%   m - 维度（2或3）
%   color - 颜色 [1×3]
%
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
a = cBest / 2;           % 半长轴
c_focal = cMin / 2;      % 半焦距
b_squared = a^2 - c_focal^2;  % b^2 = a^2 - c^2

% 数值稳定性检查
if b_squared < epsilon
    h = [];
    return;
end

b = sqrt(b_squared);

% 椭球中心（两焦点中点）
center = (focus1(1:m) + focus2(1:m)) / 2;

% 主轴方向向量
eVec = (focus2(1:m) - focus1(1:m)) / cMin;

try
    if m == 2
        % ========== 2D椭圆绘制 ==========
        theta = atan2(eVec(2), eVec(1));
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        t = linspace(0, 2*pi, 100);
        ellipse = [a*cos(t); b*sin(t)];
        rotated_ellipse = R * ellipse + center';
        
        h = plot(rotated_ellipse(1,:), rotated_ellipse(2,:), '--', ...
            'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        
    elseif m == 3
        % ========== 3D椭球绘制 ==========
        x_axis = [1; 0; 0];
        rotation_axis = cross(x_axis, eVec);
        rotation_axis_norm = norm(rotation_axis);
        
        if rotation_axis_norm < epsilon
            if dot(x_axis, eVec) > 0
                R = eye(3);
            else
                R = [-1 0 0; 0 1 0; 0 0 -1];
            end
        else
            k = rotation_axis / rotation_axis_norm;
            cos_theta = dot(x_axis, eVec);
            cos_theta = max(-1, min(1, cos_theta));
            theta = acos(cos_theta);
            
            K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
            R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
        end
        
        [phi, theta_grid] = meshgrid(linspace(0, 2*pi, 20), linspace(0, pi, 15));
        
        x_ellipsoid = a * cos(phi) .* sin(theta_grid);
        y_ellipsoid = b * sin(phi) .* sin(theta_grid);
        z_ellipsoid = b * cos(theta_grid);
        
        points = [x_ellipsoid(:)'; y_ellipsoid(:)'; z_ellipsoid(:)'];
        rotated_points = R * points + center';
        
        x_rot = reshape(rotated_points(1,:), size(x_ellipsoid));
        y_rot = reshape(rotated_points(2,:), size(y_ellipsoid));
        z_rot = reshape(rotated_points(3,:), size(z_ellipsoid));
        
        h = surf(x_rot, y_rot, z_rot, ...
            'FaceAlpha', 0.2, ...
            'EdgeColor', color, ...
            'EdgeAlpha', 0.4, ...
            'FaceColor', color, ...
            'FaceLighting', 'gouraud', ...
            'HandleVisibility', 'off');
    else
        h = [];
    end
catch ME
    warning('plotEllipsoid绘制异常: %s', ME.message);
    h = [];
end

end
