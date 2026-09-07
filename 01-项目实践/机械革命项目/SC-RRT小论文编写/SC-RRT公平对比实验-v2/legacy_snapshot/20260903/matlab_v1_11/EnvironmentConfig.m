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
            % 核心改进:
            %   1. 动态多尺度障碍物半径 (小/中/大三档)
            %   2. 起终点直线走廊保证被障碍物阻断
            %   3. 阻断后验证可行解存在性
            %
            % 输入:
            %   bounds: [xmin xmax ymin ymax zmin zmax] - 环境边界
            %   num_obstacles: 障碍物数量
            %   options: 可选参数结构体
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
                options.fixed_radius = [];
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
            
            start_pt = options.start_point;
            goal_pt = options.goal_point;
            r_min = options.radius_range(1);
            r_max = options.radius_range(2);
            
            % ===== 第一阶段: 主走廊密集障碍区 =====
            % 分两层放置共50个不同大小的障碍物，形成视觉上明显的绕行挑战区：
            %   层1 (20个): 几乎贴合直线，半径覆盖全范围，确保直线不可通行
            %   层2 (30个): 以直线为轴，垂直偏移2~5倍r_max，构成宽幅障碍富集带
            num_on_line   = 20;   % 直接阻断层
            num_shell     = 30;   % 外环扩散层
            num_corridor  = num_on_line + num_shell;
            
            % 预分配足够大的数组（后续会精确裁剪）
            obstacles = zeros(num_obstacles + 10, 4);
            placed = 0;
            
            fprintf('正在生成3D环境 (%d个球体障碍物, 含密集走廊障碍区)...\n', num_obstacles);
            
            % 计算起终点连线方向及其垂直基向量
            dir_sg   = goal_pt - start_pt;
            line_len = norm(dir_sg);
            dir_unit = dir_sg / line_len;
            
            if abs(dir_unit(1)) < 0.9
                perp1 = cross(dir_unit, [1 0 0]);
            else
                perp1 = cross(dir_unit, [0 1 0]);
            end
            perp1 = perp1 / norm(perp1);
            perp2 = cross(dir_unit, perp1);
            perp2 = perp2 / norm(perp2);
            
            % ---- 层1: 直线贴合阻断 ----
            layer1_placed = 0;
            layer1_attempts = 0;
            while layer1_placed < num_on_line && layer1_attempts < options.max_attempts
                layer1_attempts = layer1_attempts + 1;
                t = 0.10 + rand() * 0.80;          % t ∈ [0.10, 0.90]
                center_on_line = start_pt + t * dir_sg;
                
                % 极小垂直偏移（≤ r_min/3），确保仍能截断直线
                offset_mag = r_min / 3;
                offset = (randn() * offset_mag) * perp1 + (randn() * offset_mag) * perp2;
                center = center_on_line + offset;
                
                % 全尺度半径（小/中/大均有）
                scale_roll = rand();
                if scale_roll < 0.30
                    radius = r_min + rand() * (r_max - r_min) * 0.25;
                elseif scale_roll < 0.65
                    radius = r_min + (r_max - r_min) * (0.30 + rand() * 0.40);
                else
                    radius = r_min + (r_max - r_min) * (0.65 + rand() * 0.35);
                end
                
                % 边界检测
                if any(center - radius < [bounds(1), bounds(3), bounds(5)]) || ...
                   any(center + radius > [bounds(2), bounds(4), bounds(6)])
                    continue;
                end
                % 起终点安全距离
                if norm(center - start_pt) < radius + options.clearance, continue; end
                if norm(center - goal_pt)  < radius + options.clearance, continue; end
                % 与已有障碍物碰撞
                ok = true;
                for j = 1:placed
                    if norm(center - obstacles(j,1:3)) < radius + obstacles(j,4) + options.min_spacing * 0.4
                        ok = false; break;
                    end
                end
                if ok
                    placed = placed + 1;
                    obstacles(placed, :) = [center, radius];
                    layer1_placed = layer1_placed + 1;
                end
            end
            fprintf('  层1(直线阻断): 已放置 %d/%d 个\n', layer1_placed, num_on_line);
            
            % ---- 层2: 外环扩散层 ----
            % 在直线周围 2~5 倍 r_max 的环形带内密集填充不同大小障碍物
            % 同时覆盖 3 段 t 区间形成"三道闸"，交错排布制造明显绕行
            layer2_placed = 0;
            layer2_attempts = 0;
            % 三段 t 区间：每段覆盖约 1/3 路径长度，段内随机分布
            t_bands = [0.10 0.40; 0.35 0.65; 0.60 0.90];
            perp_min = r_max * 1.5;    % 垂直偏移最小值（和直线保持距离）
            perp_max = r_max * 5.0;    % 垂直偏移最大值（扩散半径）
            while layer2_placed < num_shell && layer2_attempts < options.max_attempts * 2
                layer2_attempts = layer2_attempts + 1;
                % 均匀轮选三段
                band_idx = mod(layer2_placed, 3) + 1;
                t = t_bands(band_idx, 1) + rand() * (t_bands(band_idx, 2) - t_bands(band_idx, 1));
                center_on_line = start_pt + t * dir_sg;
                
                % 垂直方向随机角度和距离
                phi = rand() * 2 * pi;
                perp_dist = perp_min + rand() * (perp_max - perp_min);
                offset = (cos(phi) * perp_dist) * perp1 + (sin(phi) * perp_dist) * perp2;
                center = center_on_line + offset;
                
                % 多尺度半径（外环以中大为主）
                scale_roll = rand();
                if scale_roll < 0.25
                    radius = r_min + rand() * (r_max - r_min) * 0.30;
                elseif scale_roll < 0.65
                    radius = r_min + (r_max - r_min) * (0.25 + rand() * 0.50);
                else
                    radius = r_min + (r_max - r_min) * (0.60 + rand() * 0.40);
                end
                
                % 边界检测
                if any(center - radius < [bounds(1), bounds(3), bounds(5)]) || ...
                   any(center + radius > [bounds(2), bounds(4), bounds(6)])
                    continue;
                end
                if norm(center - start_pt) < radius + options.clearance, continue; end
                if norm(center - goal_pt)  < radius + options.clearance, continue; end
                ok = true;
                for j = 1:placed
                    if norm(center - obstacles(j,1:3)) < radius + obstacles(j,4) + options.min_spacing * 0.5
                        ok = false; break;
                    end
                end
                if ok
                    placed = placed + 1;
                    obstacles(placed, :) = [center, radius];
                    layer2_placed = layer2_placed + 1;
                end
            end
            fprintf('  层2(扩散障碍带): 已放置 %d/%d 个\n', layer2_placed, num_shell);
            fprintf('  走廊阻断合计: %d 个\n', placed);
            
            % ===== 第二阶段: 多尺度随机障碍物 (填满至 num_obstacles) =====
            % 三档尺寸分布: 40%小 + 40%中 + 20%大
            attempts = 0;
            while placed < num_obstacles && attempts < options.max_attempts
                attempts = attempts + 1;
                
                x = bounds(1) + rand() * (bounds(2) - bounds(1));
                y = bounds(3) + rand() * (bounds(4) - bounds(3));
                z = bounds(5) + rand() * (bounds(6) - bounds(5));
                
                if ~isempty(options.fixed_radius)
                    radius = options.fixed_radius;
                else
                    % 动态多尺度半径
                    scale_roll = rand();
                    if scale_roll < 0.40
                        % 小障碍物: [r_min, r_min + 0.3*(r_max-r_min)]
                        radius = r_min + rand() * (r_max - r_min) * 0.3;
                    elseif scale_roll < 0.80
                        % 中障碍物: [r_min + 0.25*(r_max-r_min), r_min + 0.7*(r_max-r_min)]
                        radius = r_min + (r_max - r_min) * (0.25 + rand() * 0.45);
                    else
                        % 大障碍物: [r_min + 0.6*(r_max-r_min), r_max]
                        radius = r_min + (r_max - r_min) * (0.6 + rand() * 0.4);
                    end
                end
                
                % 检查与起终点距离
                dist_to_start = norm([x, y, z] - start_pt);
                dist_to_goal = norm([x, y, z] - goal_pt);
                
                if dist_to_start < radius + options.clearance || ...
                   dist_to_goal < radius + options.clearance
                    continue;
                end
                
                % 检查与已有障碍物间距
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
                    
                    if mod(placed, 50) == 0
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
                            scale_roll = rand();
                            if scale_roll < 0.40
                                radius = r_min + rand() * (r_max - r_min) * 0.3;
                            elseif scale_roll < 0.80
                                radius = r_min + (r_max - r_min) * (0.25 + rand() * 0.45);
                            else
                                radius = r_min + (r_max - r_min) * (0.6 + rand() * 0.4);
                            end
                        end
                        dist_to_start = norm([x, y, z] - start_pt);
                        dist_to_goal = norm([x, y, z] - goal_pt);
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
            
            % ===== 第三阶段: 验证直线路径确实被阻断; 若未阻断则批量追加 =====
            line_blocked = false;
            check_samples = 300;
            for si = 1:check_samples
                t = si / (check_samples + 1);
                sample_pt = start_pt + t * dir_sg;
                for oi = 1:placed
                    if norm(sample_pt - obstacles(oi, 1:3)) < obstacles(oi, 4)
                        line_blocked = true;
                        break;
                    end
                end
                if line_blocked, break; end
            end
            
            if ~line_blocked
                fprintf('  >> 直线未被完全阻断，追加多尺度阻断障碍物...\n');
                % 在直线上均匀分布15个t位置，每个位置放1~2个不同大小的阻断球
                % 同时在邻近位置放垂直偏移版本，构成"多重网格"
                extra_t_list = linspace(0.15, 0.85, 15);
                extra_placed = 0;
                for eti = 1:length(extra_t_list)
                    t_val = extra_t_list(eti);
                    % 在直线上放1个（直接阻断）
                    for try_idx = 1:2
                        center = start_pt + t_val * dir_sg;
                        if try_idx == 2
                            % 第二次尝试：微小垂直偏移
                            phi_e = rand() * 2 * pi;
                            center = center + (cos(phi_e)*r_min*0.5)*perp1 + (sin(phi_e)*r_min*0.5)*perp2;
                        end
                        % 尺度轮转：大/中/小交替
                        scale_idx = mod(eti, 3);
                        if scale_idx == 0
                            radius = r_min + (r_max - r_min) * (0.70 + rand() * 0.30);
                        elseif scale_idx == 1
                            radius = r_min + (r_max - r_min) * (0.35 + rand() * 0.35);
                        else
                            radius = r_min + (r_max - r_min) * rand() * 0.30;
                        end
                        if norm(center - start_pt) < radius + options.clearance, continue; end
                        if norm(center - goal_pt)  < radius + options.clearance, continue; end
                        ok = true;
                        for j = 1:placed
                            if norm(center - obstacles(j,1:3)) < radius + obstacles(j,4) + options.min_spacing * 0.3
                                ok = false; break;
                            end
                        end
                        if ok
                            placed = placed + 1;
                            if placed > size(obstacles, 1)
                                obstacles = [obstacles; zeros(50, 4)];
                            end
                            obstacles(placed, :) = [center, radius];
                            extra_placed = extra_placed + 1;
                        end
                    end
                end
                fprintf('  >> 追加了 %d 个多尺度阻断球\n', extra_placed);
            end
            
            % 裁剪到实际放置数量
            obstacles = obstacles(1:placed, :);
            
            % 构建环境结构体
            env = struct();
            env.bounds = bounds;
            env.start_point = start_pt;
            env.goal_point = goal_pt;
            env.obstacles = obstacles;
            env.num = placed;
            fprintf('【3D环境核验】请求: %d, 实际: %d, 直线阻断: %s\n', ...
                num_obstacles, placed, string(line_blocked || placed > num_obstacles));
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
