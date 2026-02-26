function h = plotEllipsoid(focus1, focus2, cBest, m, color)
% plotEllipsoid - 绘制超椭球体（2D椭圆或3D椭球）
%
% 功能: 
%   根据椭球体的两个焦点、焦点距离和维度参数，绘制椭圆或椭球面
%   支持2D和3D空间
%
% 输入参数:
%   focus1  - 椭球体第一个焦点 [1×m]
%   focus2  - 椭球体第二个焦点 [1×m]
%   cBest   - 椭球约束参数（从焦点1出发，到焦点2，所有椭球面上的点到两焦点距离和）
%   m       - 空间维度 (2 或 3)
%   color   - 椭球体颜色 [R G B]，范围 [0 1]
%
% 输出参数:
%   h       - 图形对象句柄（plot或surf返回的句柄）
%
% 椭球数学定义:
%   椭球上任意点P满足: ||P - focus1|| + ||P - focus2|| = cBest
%   其中 cBest >= ||focus2 - focus1|| (焦距)

% ========== 容差和输入验证 ==========
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

% ========== 计算椭球参数 ==========
% a: 半长轴（从中心到椭球边界沿主轴方向的距离）
% c: 半焦距（从中心到焦点的距离）
% b: 半短轴
a = cBest / 2;              % 半长轴 = cBest/2
c_focal = cMin / 2;         % 半焦距 = cMin/2
b_squared = a^2 - c_focal^2;  % b^2 = a^2 - c^2（椭球几何关系）

% 数值稳定性检查
if b_squared < epsilon
    h = [];
    return;
end

b = sqrt(b_squared);

% 椭球中心（两焦点中点）
center = (focus1(1:m) + focus2(1:m)) / 2;

% 主轴方向向量（从focus1指向focus2，归一化）
eVec = (focus2(1:m) - focus1(1:m)) / cMin;

try
    if m == 2
        % ========== 2D椭圆绘制 ==========
        % 在2D平面内绘制椭圆
        
        % 计算旋转角度
        theta = atan2(eVec(2), eVec(1));
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        % 参数方程生成椭圆点
        t = linspace(0, 2*pi, 100);
        ellipse = [a*cos(t); b*sin(t)];
        
        % 应用旋转和平移
        rotated_ellipse = R * ellipse + center';
        
        % 绘制椭圆
        h = plot(rotated_ellipse(1,:), rotated_ellipse(2,:), '--', ...
            'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        
    elseif m == 3
        % ========== 3D椭球绘制 ==========
        % 在3D空间内绘制椭球面
        
        % 目标：将标准椭球（长轴沿x轴）旋转到eVec方向
        
        % 标准基向量
        x_axis = [1; 0; 0];
        
        % 计算旋转轴和旋转角
        rotation_axis = cross(x_axis, eVec);
        rotation_axis_norm = norm(rotation_axis);
        
        if rotation_axis_norm < epsilon
            % eVec与x轴平行或反平行
            if dot(x_axis, eVec) > 0
                % 同向，无需旋转
                R = eye(3);
            else
                % 反向，绕y轴旋转180度
                R = [-1 0 0; 0 1 0; 0 0 -1];
            end
        else
            % 一般情况：使用Rodrigues旋转公式
            k = rotation_axis / rotation_axis_norm;  % 归一化旋转轴
            
            % 计算旋转角度（数值稳定版本）
            cos_theta = dot(x_axis, eVec);
            cos_theta = max(-1, min(1, cos_theta));  % 钳位到[-1,1]
            rot_angle = acos(cos_theta);
            
            % Rodrigues公式: R = I + sin(θ)K + (1-cos(θ))K^2
            K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];  % 反对称矩阵
            R = eye(3) + sin(rot_angle) * K + (1 - cos(rot_angle)) * (K * K);
        end
        
        % 生成标准椭球网格点
        % 在标准位置生成椭球（中心在原点，长轴沿x轴）
        [phi, theta_grid] = meshgrid(linspace(0, 2*pi, 20), linspace(0, pi, 15));
        
        % 椭球参数方程
        x_ellipsoid = a * cos(phi) .* sin(theta_grid);
        y_ellipsoid = b * sin(phi) .* sin(theta_grid);
        z_ellipsoid = b * cos(theta_grid);
        
        % 将网格点展平为3×N矩阵
        points = [x_ellipsoid(:)'; y_ellipsoid(:)'; z_ellipsoid(:)'];
        
        % 应用旋转和平移
        rotated_points = R * points + center';
        
        % 重塑为网格形式
        x_rot = reshape(rotated_points(1,:), size(x_ellipsoid));
        y_rot = reshape(rotated_points(2,:), size(y_ellipsoid));
        z_rot = reshape(rotated_points(3,:), size(z_ellipsoid));
        
        % 绘制椭球面
        h = surf(x_rot, y_rot, z_rot, ...
            'FaceAlpha', 0.2, ...          % 半透明面
            'EdgeColor', color, ...        % 边缘颜色
            'EdgeAlpha', 0.4, ...          % 边缘透明度
            'FaceColor', color, ...        % 面颜色
            'FaceLighting', 'gouraud', ... % 光照模式
            'HandleVisibility', 'off');
        
    else
        h = [];
    end
    
catch ME
    % 异常处理
    warning('plotEllipsoid绘制异常: %s\n位置: %s', ME.message, ME.stack(1).name);
    h = [];
end

end
