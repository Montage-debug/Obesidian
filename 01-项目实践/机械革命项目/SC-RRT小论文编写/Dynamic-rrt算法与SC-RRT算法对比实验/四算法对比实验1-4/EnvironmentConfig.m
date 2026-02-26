classdef EnvironmentConfig
    % EnvironmentConfig - 统一环境配置类
    % 用于生成规范的2D和3D障碍物环境
    %
    % 功能:
    %   - 生成黑色圆形障碍物(2D)
    %   - 生成黑色球体障碍物(3D)
    %   - 自动保证障碍物之间的安全距离
    %   - 确保起点和终点不与障碍物碰撞
    %   - 输出标准格式供RRT算法调用
    %
    % 作者: AI Assistant
    % 日期: 2025-12-11
    
    properties (Constant)
        DEFAULT_MIN_SPACING = 30.0;      % 障碍物之间最小间距（适配1500x1500环境）
        DEFAULT_START_GOAL_CLEARANCE = 45.0;  % 起点终点与障碍物的最小间距（适配1500x1500环境）
        MAX_PLACEMENT_ATTEMPTS = 1000;   % 放置障碍物的最大尝试次数
        COLOR_BLACK = [0 0 0];          % 黑色
    end
    
    methods (Static)
        function env = generate2DEnvironment(bounds, num_obstacles, options)
            % generate2DEnvironment - 生成2D环境配置
            %
            % 输入:
            %   bounds: [xmin xmax ymin ymax] - 环境边界
            %   num_obstacles: 障碍物数量
            %   options: 可选参数结构体
            %     .start_point: [x y] - 起点 (默认: [bounds(1)+5, bounds(3)+5])
            %     .goal_point: [x y] - 终点 (默认: [bounds(2)-5, bounds(4)-5])
            %     .radius_range: [min max] - 半径范围 (默认: [1.0 3.0])
            %     .min_spacing: 最小间距 (默认: 2.0)
            %     .clearance: 起点终点间隙 (默认: 3.0)
            %
            % 输出:
            %   env: 环境结构体
            %     .bounds: 边界
            %     .start_point: 起点
            %     .goal_point: 终点
            %     .obstacles: 障碍物信息 [x y radius]
            %     .num: 障碍物数量
            %     .dimension: '2D'
            
            % 参数解析
            if nargin < 3
                options = struct();
            end
            
            % 设置默认值
            if ~isfield(options, 'start_point')
                options.start_point = [bounds(1)+5, bounds(3)+5];
            end
            if ~isfield(options, 'goal_point')
                options.goal_point = [bounds(2)-5, bounds(4)-5];
            end
            if ~isfield(options, 'radius_range')
                options.radius_range = [1.0, 3.0];
            end
            if ~isfield(options, 'fixed_radius')
                options.fixed_radius = []; % 若设置为数值则使用固定半径
            end
            if ~isfield(options, 'min_spacing')
                options.min_spacing = EnvironmentConfig.DEFAULT_MIN_SPACING;
            end
            if ~isfield(options, 'clearance')
                options.clearance = EnvironmentConfig.DEFAULT_START_GOAL_CLEARANCE;
            end
            if ~isfield(options, 'max_attempts')
                options.max_attempts = EnvironmentConfig.MAX_PLACEMENT_ATTEMPTS;
            end
            
            % 初始化
            obstacles = zeros(num_obstacles, 3); % [x, y, radius]
            placed = 0;
            
            fprintf('正在生成2D环境 (%d个圆形障碍物)...\n', num_obstacles);
            
            % 生成障碍物
            attempts = 0;
            while placed < num_obstacles && attempts < options.max_attempts
                attempts = attempts + 1;
                
                % 随机生成位置和半径
                x = bounds(1) + rand() * (bounds(2) - bounds(1));
                y = bounds(3) + rand() * (bounds(4) - bounds(3));
                if ~isempty(options.fixed_radius)
                    radius = options.fixed_radius;
                else
                    radius = options.radius_range(1) + rand() * (options.radius_range(2) - options.radius_range(1));
                end
                
                % 检查是否与起点终点冲突
                dist_to_start = norm([x, y] - options.start_point);
                dist_to_goal = norm([x, y] - options.goal_point);
                
                if dist_to_start < radius + options.clearance || ...
                   dist_to_goal < radius + options.clearance
                    continue;
                end
                
                % 检查是否与现有障碍物冲突
                collision = false;
                for i = 1:placed
                    dist = norm([x, y] - obstacles(i, 1:2));
                    min_dist = radius + obstacles(i, 3) + options.min_spacing;
                    if dist < min_dist
                        collision = true;
                        break;
                    end
                end
                
                if ~collision
                    placed = placed + 1;
                    obstacles(placed, :) = [x, y, radius];
                    
                    if mod(placed, 10) == 0
                        fprintf('  已放置 %d/%d 个障碍物\n', placed, num_obstacles);
                    end
                end
            end
            
            if placed < num_obstacles
                % 自适应回退: 逐步减小最小间距以尽量放满
                backoff_round = 0;
                min_spacing = options.min_spacing;
                while placed < num_obstacles && backoff_round < 3
                    backoff_round = backoff_round + 1;
                    min_spacing = max(0.0, min_spacing * 0.7);
                    fill_attempts = 0;
                    while placed < num_obstacles && fill_attempts < ceil(options.max_attempts/3)
                        fill_attempts = fill_attempts + 1;
                        x = bounds(1) + rand() * (bounds(2) - bounds(1));
                        y = bounds(3) + rand() * (bounds(4) - bounds(3));
                        if ~isempty(options.fixed_radius)
                            radius = options.fixed_radius;
                        else
                            radius = options.radius_range(1) + rand() * (options.radius_range(2) - options.radius_range(1));
                        end
                        dist_to_start = norm([x, y] - options.start_point);
                        dist_to_goal = norm([x, y] - options.goal_point);
                        if dist_to_start < radius + options.clearance || dist_to_goal < radius + options.clearance
                            continue;
                        end
                        ok = true;
                        for i = 1:placed
                            dist = norm([x, y] - obstacles(i, 1:2));
                            min_dist = radius + obstacles(i, 3) + min_spacing;
                            if dist < min_dist
                                ok = false;
                                break;
                            end
                        end
                        if ok
                            placed = placed + 1;
                            obstacles(placed, :) = [x, y, radius];
                        end
                    end
                end
                if placed < num_obstacles
                    warning('只成功放置了 %d/%d 个障碍物 (空间可能不足)', placed, num_obstacles);
                    obstacles = obstacles(1:placed, :);
                else
                    fprintf('✓ 成功生成 %d 个障碍物 (自适应回退)\n', placed);
                end
            else
                fprintf('✓ 成功生成 %d 个障碍物\n', placed);
            end
            
            % 构建环境结构体
            env = struct();
            env.bounds = bounds;
            env.start_point = options.start_point;
            env.goal_point = options.goal_point;
            env.obstacles = obstacles;
            env.num = placed;
            fprintf('【2D环境核验】请求: %d, 实际: %d\n', num_obstacles, size(env.obstacles,1));
            env.dimension = '2D';
            env.x_constraints = bounds(1:2);
            env.y_constraints = bounds(3:4);
        end
        
        function env = generate3DEnvironment(bounds, num_obstacles, options)
            % generate3DEnvironment - 生成3D环境配置
            %
            % 输入:
            %   bounds: [xmin xmax ymin ymax zmin zmax] - 环境边界
            %   num_obstacles: 障碍物数量
            %   options: 可选参数结构体
            %     .start_point: [x y z] - 起点
            %     .goal_point: [x y z] - 终点
            %     .radius_range: [min max] - 半径范围 (默认: [1.0 3.0])
            %     .min_spacing: 最小间距 (默认: 2.0)
            %     .clearance: 起点终点间隙 (默认: 3.0)
            %
            % 输出:
            %   env: 环境结构体
            
            % 参数解析
            if nargin < 3
                options = struct();
            end
            
            % 设置默认值
            if ~isfield(options, 'start_point')
                options.start_point = [bounds(1)+5, bounds(3)+5, bounds(5)+5];
            end
            if ~isfield(options, 'goal_point')
                options.goal_point = [bounds(2)-5, bounds(4)-5, bounds(6)-5];
            end
            if ~isfield(options, 'radius_range')
                options.radius_range = [1.0, 3.0];
            end
            if ~isfield(options, 'fixed_radius')
                options.fixed_radius = []; % 若设置为数值则使用固定半径
            end
            if ~isfield(options, 'min_spacing')
                options.min_spacing = EnvironmentConfig.DEFAULT_MIN_SPACING;
            end
            if ~isfield(options, 'clearance')
                options.clearance = EnvironmentConfig.DEFAULT_START_GOAL_CLEARANCE;
            end
            if ~isfield(options, 'max_attempts')
                options.max_attempts = EnvironmentConfig.MAX_PLACEMENT_ATTEMPTS;
            end
            
            % 初始化
            obstacles = zeros(num_obstacles, 4); % [x, y, z, radius]
            placed = 0;
            
            fprintf('正在生成3D环境 (%d个球体障碍物)...\n', num_obstacles);
            
            % 生成障碍物
            attempts = 0;
            while placed < num_obstacles && attempts < options.max_attempts
                attempts = attempts + 1;
                
                % 随机生成位置和半径
                x = bounds(1) + rand() * (bounds(2) - bounds(1));
                y = bounds(3) + rand() * (bounds(4) - bounds(3));
                z = bounds(5) + rand() * (bounds(6) - bounds(5));
                if ~isempty(options.fixed_radius)
                    radius = options.fixed_radius;
                else
                    radius = options.radius_range(1) + rand() * (options.radius_range(2) - options.radius_range(1));
                end
                
                % 检查是否与起点终点冲突
                dist_to_start = norm([x, y, z] - options.start_point);
                dist_to_goal = norm([x, y, z] - options.goal_point);
                
                if dist_to_start < radius + options.clearance || ...
                   dist_to_goal < radius + options.clearance
                    continue;
                end
                
                % 检查是否与现有障碍物冲突
                collision = false;
                for i = 1:placed
                    dist = norm([x, y, z] - obstacles(i, 1:3));
                    min_dist = radius + obstacles(i, 4) + options.min_spacing;
                    if dist < min_dist
                        collision = true;
                        break;
                    end
                end
                
                if ~collision
                    placed = placed + 1;
                    obstacles(placed, :) = [x, y, z, radius];
                    
                    if mod(placed, 10) == 0
                        fprintf('  已放置 %d/%d 个障碍物\n', placed, num_obstacles);
                    end
                end
            end
            
            if placed < num_obstacles
                % 自适应回退: 逐步减小最小间距以尽量放满
                backoff_round = 0;
                min_spacing = options.min_spacing;
                while placed < num_obstacles && backoff_round < 3
                    backoff_round = backoff_round + 1;
                    min_spacing = max(0.0, min_spacing * 0.7);
                    fill_attempts = 0;
                    while placed < num_obstacles && fill_attempts < ceil(options.max_attempts/3)
                        fill_attempts = fill_attempts + 1;
                        x = bounds(1) + rand() * (bounds(2) - bounds(1));
                        y = bounds(3) + rand() * (bounds(4) - bounds(3));
                        z = bounds(5) + rand() * (bounds(6) - bounds(5));
                        if ~isempty(options.fixed_radius)
                            radius = options.fixed_radius;
                        else
                            radius = options.radius_range(1) + rand() * (options.radius_range(2) - options.radius_range(1));
                        end
                        % 确保使用3D坐标计算距离
                        start_3d = options.start_point;
                        if length(start_3d) == 2
                            start_3d = [start_3d(1), start_3d(2), (bounds(5)+bounds(6))/2];
                        end
                        goal_3d = options.goal_point;
                        if length(goal_3d) == 2
                            goal_3d = [goal_3d(1), goal_3d(2), (bounds(5)+bounds(6))/2];
                        end
                        dist_to_start = norm([x, y, z] - start_3d);
                        dist_to_goal = norm([x, y, z] - goal_3d);
                        if dist_to_start < radius + options.clearance || dist_to_goal < radius + options.clearance
                            continue;
                        end
                        ok = true;
                        for i = 1:placed
                            dist = norm([x, y, z] - obstacles(i, 1:3));
                            min_dist = radius + obstacles(i, 4) + min_spacing;
                            if dist < min_dist
                                ok = false;
                                break;
                            end
                        end
                        if ok
                            placed = placed + 1;
                            obstacles(placed, :) = [x, y, z, radius];
                        end
                    end
                end
                if placed < num_obstacles
                    warning('只成功放置了 %d/%d 个障碍物 (空间可能不足)', placed, num_obstacles);
                    obstacles = obstacles(1:placed, :);
                else
                    fprintf('✓ 成功生成 %d 个障碍物 (自适应回退)\n', placed);
                end
            else
                fprintf('✓ 成功生成 %d 个障碍物\n', placed);
            end
            
            % 构建环境结构体
            env = struct();
            env.bounds = bounds;
            env.start_point = options.start_point;
            env.goal_point = options.goal_point;
            env.obstacles = obstacles;
            env.num = placed;
            fprintf('【3D环境核验】请求: %d, 实际: %d\n', num_obstacles, size(env.obstacles,1));
            env.dimension = '3D';
            env.x_constraints = bounds(1:2);
            env.y_constraints = bounds(3:4);
            env.z_constraints = bounds(5:6);
        end
        
        function saveEnvironment(env, filename)
            % saveEnvironment - 保存环境配置到.mat文件
            %
            % 输入:
            %   env: 环境结构体
            %   filename: 保存的文件名
            
            % 转换为RRT工具箱兼容格式
            num = env.num;
            x_constraints = env.x_constraints;
            y_constraints = env.y_constraints;
            
            if strcmp(env.dimension, '2D')
                % 2D: 将圆形转换为多边形近似
                output = cell(num, 1);
                for i = 1:num
                    cx = env.obstacles(i, 1);
                    cy = env.obstacles(i, 2);
                    r = env.obstacles(i, 3);
                    
                    % 用32边多边形近似圆
                    theta = linspace(0, 2*pi, 33);
                    vertices = [cx + r*cos(theta)', cy + r*sin(theta)'];
                    output{i} = vertices;
                end
            else
                % 3D: 保存球体参数
                output = cell(num, 1);
                z_constraints = env.z_constraints;
                for i = 1:num
                    output{i} = struct(...
                        'center', env.obstacles(i, 1:3), ...
                        'radius', env.obstacles(i, 4), ...
                        'type', 'sphere');
                end
            end
            
            % 保存
            if strcmp(env.dimension, '3D')
                save(filename, 'num', 'output', 'x_constraints', 'y_constraints', 'z_constraints');
            else
                save(filename, 'num', 'output', 'x_constraints', 'y_constraints');
            end
            
            fprintf('✓ 环境已保存至: %s\n', filename);
        end
        
        function visualize2D(env, show_labels)
            % visualize2D - 可视化2D环境
            %
            % 输入:
            %   env: 环境结构体
            %   show_labels: 是否显示标签 (默认: true)
            
            if nargin < 2
                show_labels = true;
            end
            
            figure('Color', 'w');
            hold on; axis equal; grid on;
            
            % 绘制边界
            xlim([env.bounds(1), env.bounds(2)]);
            ylim([env.bounds(3), env.bounds(4)]);
            
            % 绘制障碍物
            for i = 1:env.num
                x = env.obstacles(i, 1);
                y = env.obstacles(i, 2);
                r = env.obstacles(i, 3);
                
                rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
                         'Curvature', [1 1], ...
                         'FaceColor', EnvironmentConfig.COLOR_BLACK, ...
                         'EdgeColor', 'none');
            end
            
            % 绘制起点和终点
            plot(env.start_point(1), env.start_point(2), 'go', ...
                 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
            plot(env.goal_point(1), env.goal_point(2), 'r^', ...
                 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
            
            if show_labels
                text(env.start_point(1), env.start_point(2), ' 起点', ...
                     'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
                text(env.goal_point(1), env.goal_point(2), ' 终点', ...
                     'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
            end
            
            xlabel('X (m)');
            ylabel('Y (m)');
            title(sprintf('2D环境 - %d个障碍物', env.num));
            legend('障碍物', '起点', '终点', 'Location', 'best');
        end
        
        function visualize3D(env, show_labels)
            % visualize3D - 可视化3D环境
            %
            % 输入:
            %   env: 环境结构体
            %   show_labels: 是否显示标签 (默认: true)
            
            if nargin < 2
                show_labels = true;
            end
            
            figure('Color', 'w');
            hold on; axis equal; grid on;
            
            % 设置视角
            view(3);
            
            % 设置边界
            xlim([env.bounds(1), env.bounds(2)]);
            ylim([env.bounds(3), env.bounds(4)]);
            zlim([env.bounds(5), env.bounds(6)]);
            
            % 绘制障碍物 (球体)
            [X, Y, Z] = sphere(20);
            for i = 1:env.num
                cx = env.obstacles(i, 1);
                cy = env.obstacles(i, 2);
                cz = env.obstacles(i, 3);
                r = env.obstacles(i, 4);
                
                surf(X*r + cx, Y*r + cy, Z*r + cz, ...
                     'FaceColor', EnvironmentConfig.COLOR_BLACK, ...
                     'EdgeColor', 'none', ...
                     'FaceAlpha', 0.8);
            end
            
            % 绘制起点和终点
            plot3(env.start_point(1), env.start_point(2), env.start_point(3), ...
                  'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
            plot3(env.goal_point(1), env.goal_point(2), env.goal_point(3), ...
                  'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
            
            if show_labels
                text(env.start_point(1), env.start_point(2), env.start_point(3), ...
                     ' 起点', 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
                text(env.goal_point(1), env.goal_point(2), env.goal_point(3), ...
                     ' 终点', 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
            end
            
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            title(sprintf('3D环境 - %d个障碍物', env.num));
            legend('障碍物', '起点', '终点', 'Location', 'best');
            
            % 添加光照效果
            lighting gouraud;
            camlight;
        end
        
        function result = checkCollision2D(env, point, safety_margin)
            % checkCollision2D - 检查2D点是否与障碍物碰撞
            %
            % 输入:
            %   env: 环境结构体
            %   point: [x y] - 待检查的点
            %   safety_margin: 安全余量 (默认: 0)
            %
            % 输出:
            %   result: true-碰撞, false-无碰撞
            
            if nargin < 3
                safety_margin = 0;
            end
            
            result = false;
            for i = 1:env.num
                dist = norm(point - env.obstacles(i, 1:2));
                if dist < env.obstacles(i, 3) + safety_margin
                    result = true;
                    return;
                end
            end
        end
        
        function result = checkCollision3D(env, point, safety_margin)
            % checkCollision3D - 检查3D点是否与障碍物碰撞
            %
            % 输入:
            %   env: 环境结构体
            %   point: [x y z] - 待检查的点
            %   safety_margin: 安全余量 (默认: 0)
            %
            % 输出:
            %   result: true-碰撞, false-无碰撞
            
            if nargin < 3
                safety_margin = 0;
            end
            
            result = false;
            for i = 1:env.num
                dist = norm(point - env.obstacles(i, 1:3));
                if dist < env.obstacles(i, 4) + safety_margin
                    result = true;
                    return;
                end
            end
        end
    end
end
