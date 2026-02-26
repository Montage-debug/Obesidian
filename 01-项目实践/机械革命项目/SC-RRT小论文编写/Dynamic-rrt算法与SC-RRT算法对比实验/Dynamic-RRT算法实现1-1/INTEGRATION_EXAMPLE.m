% =========================================================================
%     椭球体显示集成示例 - 修改 single_run_comparison.m
% =========================================================================
% 
% 本文件展示如何修改 single_run_comparison.m 来集成椭球体显示功能
% 
% 将以下代码复制粘贴到 single_run_comparison.m 中的相应位置
%
% =========================================================================

% ========== 修改位置 1: 在运行算法循环中 ==========
% 
% 原始位置: single_run_comparison.m 第 165-230 行
% 
% 原始代码:
% --------
%     elseif strcmp(alg.type, 'sc_rrt')
%         % 运行SC-RRT
%         fig_temp = figure('Visible', 'off');
%         
%         tic;
%         [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
%             config.startPoint, ...
%             config.goalPoint, ...
%             config.bounds, ...
%             obstacles, ...
%             fig_temp, '', 0, 0, ...
%             'Mode', alg.params.Mode, ...
%             'MaxIterations', config.max_iterations, ...
%             'UseParetoFrontier', alg.params.UseParetoFrontier, ...
%             'EnableVisualization', false);
%         elapsed_time = toc;

% 修改代码:
% ---------

    elseif strcmp(alg.type, 'sc_rrt')
        % ========== 椭球体显示配置 ==========
        % 是否启用椭球体动态显示 (true: 启用, false: 禁用)
        enable_ellipsoid_display = true;
        
        if enable_ellipsoid_display
            % 创建椭球体显示窗口
            fig_ellipsoid = figure('Name', [alg.name ' - 椭球体约束动态显示'], ...
                                  'Position', [950 100 900 800], ...
                                  'Color', 'w');
            
            % 初始化显示窗口
            hold on; axis equal; grid on;
            xlim(config.bounds(1:2));
            ylim(config.bounds(3:4));
            xlabel('X (mm)', 'FontSize', 12);
            ylabel('Y (mm)', 'FontSize', 12);
            title([alg.name ' - 椭球体约束动态显示'], 'FontSize', 13, 'FontWeight', 'bold');
            
            % 绘制障碍物
            fprintf('  正在绘制障碍物...\n');
            for i = 1:size(obstacles.circles, 1)
                circle = obstacles.circles(i, :);
                rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                                      2*circle(3), 2*circle(3)], ...
                         'Curvature', [1 1], ...
                         'FaceColor', [0.3 0.3 0.3], ...
                         'EdgeColor', [0.3 0.3 0.3], ...
                         'LineWidth', 0.5);
            end
            
            % 绘制起点和目标点
            plot(config.startPoint(1), config.startPoint(2), 'go', ...
                 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
            plot(config.goalPoint(1), config.goalPoint(2), 'rs', ...
                 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
            
            legend('起点', '目标点', 'Location', 'best');
            drawnow limitrate;
            
            % 使用显示窗口作为算法窗口
            fig_temp = fig_ellipsoid;
        else
            fig_temp = figure('Visible', 'off');
        end
        
        % 运行SC-RRT
        fprintf('  运行 SC-RRT 算法...\n');
        tic;
        [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
            config.startPoint, ...
            config.goalPoint, ...
            config.bounds, ...
            obstacles, ...
            fig_temp, '', 0, 0, ...
            'Mode', alg.params.Mode, ...
            'MaxIterations', config.max_iterations, ...
            'UseParetoFrontier', alg.params.UseParetoFrontier, ...
            'EnableVisualization', enable_ellipsoid_display, ...
            'VisualizationInterval', 50);  % 每50次迭代更新一次
        elapsed_time = toc;
        
        % 清理
        if ~enable_ellipsoid_display
            close(fig_temp);
        end


% ========== 修改位置 2: 在单个图结果显示中 ==========
% 
% 原始位置: single_run_comparison.m 第 265-310 行 (绘制算法对比结果)
% 
% 在 subplot(num_rows, num_cols, alg_idx); 后添加椭球体:
% 

% 添加代码 (在绘制路径后):
    if strcmp(alg.type, 'sc_rrt') && result.success
        % 添加椭球体显示到对比结果图中
        try
            % 计算最后的交汇点
            if isfield(result, 'treeA') && isfield(result, 'treeB')
                m = 2;  % 2D环境
                centroidA = mean(result.treeA(1:min(50,size(result.treeA,1)), 1:m), 1);
                centroidB = mean(result.treeB(1:min(50,size(result.treeB,1)), 1:m), 1);
                meetPoint = (centroidA + centroidB) / 2;
                
                % 创建树结构用于椭球参数计算
                treeA_full = [result.treeA, ones(size(result.treeA,1),1), ...
                             zeros(size(result.treeA,1),1), zeros(size(result.treeA,1),1)];
                treeB_full = [result.treeB, ones(size(result.treeB,1),1), ...
                             zeros(size(result.treeB,1),1), zeros(size(result.treeB,1),1)];
                
                % 计算椭球参数
                [cA, cB, cmA, cmB] = calculateDualEllipsoidParams(...
                    treeA_full, treeB_full, config.startPoint, config.goalPoint, meetPoint, m, 1.2);
                
                % 绘制椭球体
                if isfinite(cA) && cA > cmA * 1.02
                    plotEllipsoid(config.startPoint, meetPoint, cA, m, [0.8 0.3 0.3]);
                end
                if isfinite(cB) && cB > cmB * 1.02
                    plotEllipsoid(meetPoint, config.goalPoint, cB, m, [0.3 0.3 0.8]);
                end
                
                % 标记交汇点
                scatter(meetPoint(1), meetPoint(2), 150, 'y', 'filled', 'd', ...
                       'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
            end
        catch ME
            % 如果出错，仅显示路径，忽略椭球体
            fprintf('    ⚠ 椭球体绘制出错: %s\n', ME.message);
        end
    end


% ========== 修改位置 3: 添加导入语句 ==========
% 
% 在 single_run_comparison.m 的最开始添加:
% 

% 确保所有函数都在路径中
addpath(pwd);  % 添加当前目录到路径


% =========================================================================
% 使用说明
% =========================================================================
%
% 1. 启用/禁用椭球体显示:
%    在上面的代码中找到: enable_ellipsoid_display = true;
%    改为 false 可以禁用椭球体显示
%
% 2. 调整更新频率:
%    修改参数: 'VisualizationInterval', 50
%    数值越大，更新越不频繁（性能更好）
%    数值越小，显示越流畅（性能开销更大）
%
% 3. 椭球体颜色定制:
%    plotEllipsoid(..., [0.8 0.3 0.3])  % 红色
%    plotEllipsoid(..., [0.3 0.3 0.8])  % 蓝色
%    可修改RGB值自定义颜色
%
% =========================================================================
