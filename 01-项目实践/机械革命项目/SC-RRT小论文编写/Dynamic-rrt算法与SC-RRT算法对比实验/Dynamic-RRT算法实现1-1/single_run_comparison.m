% =========================================================================
%     单次运行对比实验 - 论文图片生成
% =========================================================================
% 目的: 单次运行所有算法，生成高质量论文图片
% 输出: 两个对话框 - (1) 障碍物环境 (2) 算法对比结果
% =========================================================================
clear; clc; close all;

%% =================== 环境配置 (论文标准) ===================
fprintf('\n========================================\n');
fprintf('单次运行对比实验 - 论文图片生成\n');
fprintf('========================================\n\n');

% =========================================================================
% 维度切换: 将 config.dimension 改为 '3D' 即可切换到三维空间
% =========================================================================
config = struct();
config.dimension = '2D';   % ← 修改为 '3D' 即可切换三维模式

% ----- 2D 配置 -----
if strcmp(config.dimension, '2D')
    config.bounds        = [0 1500 0 1500];
    config.startPoint    = [400 400];
    config.goalPoint     = [1100 1100];
    config.numObstacles  = 225;
    config.obstacleRadius = 15;
% ----- 3D 配置 -----
else
    config.bounds        = [0 1500 0 1500 0 1500];
    config.startPoint    = [200 200 200];
    config.goalPoint     = [1300 1300 1300];
    config.numObstacles  = 400;
    config.obstacleRadius = 40;
end

config.seed           = 42;
config.max_iterations = 10000;
config.time_limit     = 30;

% =========================================================================
% 障碍物随机种子: 每次运行生成不同的障碍物环境
% 设为 'shuffle' 则每次随机; 设为固定数字(如42)则可复现
% =========================================================================
config.obstacleSeed = 'shuffle';   % ← 改为数字(如42)可固定环境

% 维度标志（方便后续判断）
is3D = strcmp(config.dimension, '3D');

fprintf('环境配置 (论文标准):\n');
if is3D
    fprintf('  空间: [%d %d] × [%d %d] × [%d %d] mm\n', config.bounds);
    fprintf('  起点: (%.0f, %.0f, %.0f)\n', config.startPoint);
    fprintf('  终点: (%.0f, %.0f, %.0f)\n', config.goalPoint);
else
    fprintf('  空间: [%d %d] × [%d %d] mm\n', config.bounds);
    fprintf('  起点: (%.0f, %.0f)\n', config.startPoint);
    fprintf('  终点: (%.0f, %.0f)\n', config.goalPoint);
end
fprintf('  障碍物: %d 个 (半径 %d mm)\n', config.numObstacles, config.obstacleRadius);
fprintf('  最大迭代: %d\n', config.max_iterations);
fprintf('========================================\n\n');

%% =================== 生成障碍物环境 ===================
fprintf('正在生成障碍物环境...\n');

% ===== 关键：确保调用本目录的函数，而非其他路径的同名旧版 =====
thisDir = fileparts(mfilename('fullpath'));   % 获取本脚本所在目录的绝对路径
addpath(thisDir);                             % 加到 MATLAB 搜索路径最前面
cd(thisDir);                                  % 切换工作目录
rehash path;                                  % 清除函数缓存
fprintf('  工作目录: %s\n', thisDir);
fprintf('  which generateObstacles → %s\n', which('generateObstacles'));

% 障碍物环境用独立种子：每次运行随机，但同一次运行内所有算法共享
if ischar(config.obstacleSeed) && strcmp(config.obstacleSeed, 'shuffle')
    rng('shuffle');
    actualSeed = rng;
    fprintf('  障碍物种子: 随机 (state=%d)\n', actualSeed.Seed);
else
    rng(config.obstacleSeed);
    fprintf('  障碍物种子: %d (固定)\n', config.obstacleSeed);
end

obstacles = generateObstacles(config.dimension, ...
                             config.bounds, ...
                             config.numObstacles, ...
                             config.obstacleRadius, ...
                             config.startPoint, ...
                             config.goalPoint);
fprintf('✓ 障碍物生成器（本地版本）\n');
if is3D
    actualCount = size(obstacles.spheres, 1);
else
    actualCount = size(obstacles.circles, 1);
end
fprintf('✓ 障碍物生成完成: %d 个 (目标 %d)\n\n', actualCount, config.numObstacles);
if actualCount ~= config.numObstacles
    warning('障碍物数量 %d ≠ 目标 %d，请检查 generateObstacles.m', actualCount, config.numObstacles);
end

%% =================== 图1: 显示障碍物环境 ===================
fig1 = figure('Name', '障碍物环境 - 论文标准配置', ...
              'Position', [100 100 800 700], ...
              'Color', 'w');

hold on; axis equal; grid on;
xlim(config.bounds(1:2));
ylim(config.bounds(3:4));
xlabel('X (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y (mm)', 'FontSize', 12, 'FontWeight', 'bold');

if is3D
    zlim(config.bounds(5:6));
    zlabel('Z (mm)', 'FontSize', 12, 'FontWeight', 'bold');
    view(3);
    title(sprintf('障碍物环境 (%s, %d障碍物)', config.dimension, config.numObstacles), ...
          'FontSize', 14, 'FontWeight', 'bold');
    % 绘制3D球形障碍物
    for i = 1:size(obstacles.spheres, 1)
        sp = obstacles.spheres(i, :);
        [Xs,Ys,Zs] = sphere(12);
        surf(Xs*sp(4)+sp(1), Ys*sp(4)+sp(2), Zs*sp(4)+sp(3), ...
             'FaceColor', [0 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    end
    plot3(config.startPoint(1), config.startPoint(2), config.startPoint(3), 'go', ...
          'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot3(config.goalPoint(1), config.goalPoint(2), config.goalPoint(3), 'rs', ...
          'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    text(config.startPoint(1), config.startPoint(2), config.startPoint(3)+60, '起点', ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    text(config.goalPoint(1), config.goalPoint(2), config.goalPoint(3)+60, '终点', ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
else
    title(sprintf('障碍物环境 (论文标准: 1500×1500mm, %d障碍物)', config.numObstacles), ...
          'FontSize', 14, 'FontWeight', 'bold');
    % 绘制2D圆形障碍物
    for i = 1:size(obstacles.circles, 1)
        circle = obstacles.circles(i, :);
        rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                              2*circle(3), 2*circle(3)], ...
                 'Curvature', [1 1], ...
                 'FaceColor', [0 0 0], ...
                 'EdgeColor', [0 0 0], ...
                 'LineWidth', 0.5);
    end
    plot(config.startPoint(1), config.startPoint(2), 'go', ...
         'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(config.goalPoint(1), config.goalPoint(2), 'rs', ...
         'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    text(config.startPoint(1), config.startPoint(2)-50, '起点', ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
    text(config.goalPoint(1), config.goalPoint(2)+50, '终点', ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
end

legend('障碍物', '起点', '终点', 'Location', 'best', 'FontSize', 10);
set(gca, 'FontSize', 11);

fprintf('✓ 图1: 障碍物环境已显示\n\n');

%% =================== 定义测试算法 ===================
algorithms = {};

% 1. Dynamic RRT (Interval=4)
% 2. [1 0 0]是红色
algorithms{end+1} = struct(...
    'name', 'Dynamic-RRT (I=4)', ...
    'type', 'dynamic_rrt', ...
    'params', struct('Interval', 4, 'ParetoProb', 0.1), ...
    'color', [1 0 0], ...     
    'linestyle', '-', ...
    'linewidth', 2.5 ...
);

% % 2. Dynamic RRT (Interval=8)
% algorithms{end+1} = struct(...
%     'name', 'Dynamic-RRT (I=8)', ...
%     'type', 'dynamic_rrt', ...
%     'params', struct('Interval', 8, 'ParetoProb', 0.1), ...
%     'color', [0.93 0.69 0.13], ...
%     'linestyle', '-', ...
%     'linewidth', 2.5 ...
% );

% % 3. SC-RRT Basic
% algorithms{end+1} = struct(...
%     'name', 'SC-RRT Basic', ...
%     'type', 'sc_rrt', ...
%     'params', struct('Mode', 'basic', 'UseParetoFrontier', false), ...
%     'color', [0.00 0.45 0.74], ...
%     'linestyle', '-', ...
%     'linewidth', 2.5 ...
% );

% 4. SC-RRT with PID
algorithms{end+1} = struct(...
    'name', 'SC-RRT', ...
    'type', 'sc_rrt', ...
    'params', struct('Mode', 'pid', 'UseParetoFrontier', false), ...
    'color', [0 0 1], ...
    'linestyle', '-', ...
    'linewidth', 2.5 ...
);

% % 5. SC-RRT Adaptive (完整版)
% algorithms{end+1} = struct(...
%     'name', 'SC-RRT Adaptive', ...
%     'type', 'sc_rrt', ...
%     'params', struct('Mode', 'adaptive', 'UseParetoFrontier', true), ...
%     'color', [0.49 0.18 0.56], ...
%     'linestyle', '-', ...
%     'linewidth', 2.5 ...
% );

num_algorithms = length(algorithms);

%% =================== 运行所有算法 ===================
fprintf('========================================\n');
fprintf('开始运行 %d 个算法...\n', num_algorithms);
fprintf('========================================\n\n');

results = cell(num_algorithms, 1);

for alg_idx = 1:num_algorithms
    alg = algorithms{alg_idx};
    
    fprintf('【算法 %d/%d】 %s\n', alg_idx, num_algorithms, alg.name);
    fprintf('  类型: %s\n', alg.type);
    
    % 设置随机种子（与障碍物生成不同，但可复现）
    rng(config.seed + 100 + alg_idx);
    
    try
        if strcmp(alg.type, 'dynamic_rrt')
            % 运行Dynamic RRT
            tic;
            [tree, path, success, metrics] = DynamicRRT(...
                config.startPoint, ...
                config.goalPoint, ...
                config.bounds, ...
                obstacles, ...
                'MaxIterations', config.max_iterations, ...
                'Interval', alg.params.Interval, ...
                'ParetoProb', alg.params.ParetoProb, ...
                'EnableVisualization', true);
            elapsed_time = toc;
            
            if success
                results{alg_idx} = struct();
                results{alg_idx}.success = true;
                results{alg_idx}.tree = tree;
                results{alg_idx}.path = path;
                results{alg_idx}.time = elapsed_time;
                results{alg_idx}.length = metrics.pathLength;
                results{alg_idx}.nodes = metrics.nodeCount;
                results{alg_idx}.iterations = metrics.iterations;
                results{alg_idx}.smoothness = calculateSmoothness(path);
                
                fprintf('  ✓ 成功!\n');
                fprintf('    时间: %.4f 秒\n', elapsed_time);
                fprintf('    路径长度: %.2f mm\n', metrics.pathLength);
                fprintf('    节点数: %d\n', metrics.nodeCount);
                fprintf('    迭代次数: %d\n', metrics.iterations);
            else
                fprintf('  ✗ 失败 (未找到路径)\n');
                results{alg_idx} = struct('success', false);
            end
            
        elseif strcmp(alg.type, 'sc_rrt')
            % ========== 椭球体动态显示配置 ==========
            % 设置 enable_ellipsoid_display = true 可开启椭球体动态显示
            % 椭球体将在独立窗口中实时更新，显示SC-RRT的采样约束范围
            enable_ellipsoid_display = true;
            
            if enable_ellipsoid_display
                % 创建独立的椭球体显示窗口
                fig_temp = figure('Name', [alg.name ' - 超椭球体约束动态显示'], ...
                                  'Position', [950 100 900 800], ...
                                  'Color', 'w', ...
                                  'Renderer', 'opengl');
                
                hold on; axis equal; grid on;
                xlim(config.bounds(1:2));
                ylim(config.bounds(3:4));
                xlabel('X (mm)', 'FontSize', 12, 'FontWeight', 'bold');
                ylabel('Y (mm)', 'FontSize', 12, 'FontWeight', 'bold');
                title([alg.name ' - 超椭球体采样约束实时显示'], ...
                      'FontSize', 13, 'FontWeight', 'bold');
                
                if is3D
                    zlim(config.bounds(5:6));
                    zlabel('Z (mm)', 'FontSize', 12, 'FontWeight', 'bold');
                    view(3);
                    % 绘制3D球形障碍物
                    for i = 1:size(obstacles.spheres, 1)
                        sp = obstacles.spheres(i, :);
                        [Xs,Ys,Zs] = sphere(10);
                        surf(Xs*sp(4)+sp(1), Ys*sp(4)+sp(2), Zs*sp(4)+sp(3), ...
                             'FaceColor', [0.25 0.25 0.25], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
                    end
                    plot3(config.startPoint(1), config.startPoint(2), config.startPoint(3), 'go', ...
                          'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2.5, 'DisplayName', '起点');
                    plot3(config.goalPoint(1), config.goalPoint(2), config.goalPoint(3), 'rs', ...
                          'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2.5, 'DisplayName', '终点');
                else
                    % 绘制2D障碍物（深灰色）
                    for i = 1:size(obstacles.circles, 1)
                        circle = obstacles.circles(i, :);
                        rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                                              2*circle(3), 2*circle(3)], ...
                                 'Curvature', [1 1], ...
                                 'FaceColor', [0.25 0.25 0.25], ...
                                 'EdgeColor', [0.25 0.25 0.25], ...
                                 'LineWidth', 0.5);
                    end
                    % 绘制起点（绿色）和终点（红色）
                    plot(config.startPoint(1), config.startPoint(2), 'go', ...
                         'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2.5, ...
                         'DisplayName', '起点');
                    plot(config.goalPoint(1), config.goalPoint(2), 'rs', ...
                         'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2.5, ...
                         'DisplayName', '终点');
                    % 添加颜色说明
                    text(config.bounds(2) * 0.05, config.bounds(4) * 0.95, ...
                         {'椭球体颜色说明:', ...
                          '● 绿色: 起点→交汇点', ...
                          '● 蓝色: 交汇点→终点', ...
                          '◆ 黄色: 动态交汇点'}, ...
                         'FontSize', 10, 'FontWeight', 'bold', ...
                         'VerticalAlignment', 'top', ...
                         'BackgroundColor', [1 1 1 0.8]);
                end
                
                legend('起点', '终点', 'Location', 'southeast');
                drawnow limitrate;
                
                fprintf('  ✓ 椭球体显示窗口已创建\n');
                fprintf('  运行SC-RRT算法（带椭球体动态显示）...\n');
            else
                fig_temp = figure('Visible', 'off');
            end
            
            tic;
            [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
                config.startPoint, ...
                config.goalPoint, ...
                config.bounds, ...
                obstacles, ...
                fig_temp, '', 0, 0, ...
                'Mode', alg.params.Mode, ...
                'MaxIterations', config.max_iterations, ...
                'UpdateInterval', 1, ...              % 每次迭代都重算交汇点并刷新椭球（与copilot-1.6 main.m一致）
                'SmoothingFactor', 0.9, ...           % 交汇点平滑因子
                'EllipsoidBuffer', 2.0, ...           % 椭球缓冲系数
                'VisualizeDualEllipsoid', enable_ellipsoid_display, ...
                'UseParetoFrontier', alg.params.UseParetoFrontier, ...
                'EnableVisualization', enable_ellipsoid_display, ...
                'VisualizationInterval', 5);          % 节点绘制间隔（与copilot-1.6一致）
            elapsed_time = toc;
            
            % 如果椭球体显示已打开，保留窗口；否则关闭
            if ~enable_ellipsoid_display
                close(fig_temp);
            end
            
            if success && ~isempty(path)
                results{alg_idx} = struct();
                results{alg_idx}.success = true;
                results{alg_idx}.treeA = treeA;
                results{alg_idx}.treeB = treeB;
                results{alg_idx}.path = path;
                
                % 处理字段名兼容性
                if isfield(metrics, 'computationTime')
                    results{alg_idx}.time = metrics.computationTime;
                elseif isfield(metrics, 'planningTime')
                    results{alg_idx}.time = metrics.planningTime;
                else
                    results{alg_idx}.time = elapsed_time;
                end
                
                if isfield(metrics, 'pathLength')
                    results{alg_idx}.length = metrics.pathLength;
                elseif isfield(metrics, 'finalPathLength')
                    results{alg_idx}.length = metrics.finalPathLength;
                else
                    results{alg_idx}.length = calculatePathLength(path);
                end
                
                if isfield(metrics, 'totalNodes')
                    results{alg_idx}.nodes = metrics.totalNodes;
                else
                    results{alg_idx}.nodes = treeA.count + treeB.count;
                end
                
                if isfield(metrics, 'iterations')
                    results{alg_idx}.iterations = metrics.iterations;
                elseif isfield(metrics, 'totalIterations')
                    results{alg_idx}.iterations = metrics.totalIterations;
                else
                    results{alg_idx}.iterations = NaN;
                end
                
                results{alg_idx}.smoothness = calculateSmoothness(path);
                
                fprintf('  ✓ 成功!\n');
                fprintf('    时间: %.4f 秒\n', results{alg_idx}.time);
                fprintf('    路径长度: %.2f mm\n', results{alg_idx}.length);
                fprintf('    节点数: %d\n', results{alg_idx}.nodes);
                if ~isnan(results{alg_idx}.iterations)
                    fprintf('    迭代次数: %d\n', results{alg_idx}.iterations);
                end
            else
                fprintf('  ✗ 失败 (未找到路径)\n');
                results{alg_idx} = struct('success', false);
            end
        end
        
    catch ME
        fprintf('  ✗ 错误: %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('    位置: %s (Line %d)\n', ME.stack(1).name, ME.stack(1).line);
        end
        results{alg_idx} = struct('success', false);
    end
    
    fprintf('\n');
end

fprintf('========================================\n');
fprintf('所有算法运行完成!\n');
fprintf('========================================\n\n');

%% =================== 图2: 算法对比结果 ===================
fig2 = figure('Name', '算法对比结果 - 路径规划对比', ...
              'Position', [100 100 1600 700], ...
              'Color', 'w');

% 创建1×2子图布局（左右分布）
num_rows = 1;
num_cols = 2;

for alg_idx = 1:num_algorithms
    alg = algorithms{alg_idx};
    result = results{alg_idx};
    
    subplot(num_rows, num_cols, alg_idx);
    hold on; axis equal; grid on;
    xlim(config.bounds(1:2));
    ylim(config.bounds(3:4));
    
    % 标题将在后面根据成功/失败状态设置
    xlabel('X (mm)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Y (mm)', 'FontSize', 13, 'FontWeight', 'bold');
    
    if is3D
        zlim(config.bounds(5:6));
        zlabel('Z (mm)', 'FontSize', 13, 'FontWeight', 'bold');
        view(3);
        % 绘制3D球形障碍物
        for i = 1:size(obstacles.spheres, 1)
            sp = obstacles.spheres(i, :);
            [Xs,Ys,Zs] = sphere(8);
            surf(Xs*sp(4)+sp(1), Ys*sp(4)+sp(2), Zs*sp(4)+sp(3), ...
                 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        end
        plot3(config.startPoint(1), config.startPoint(2), config.startPoint(3), 'go', ...
              'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
        plot3(config.goalPoint(1), config.goalPoint(2), config.goalPoint(3), 'rs', ...
              'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    else
        % 绘制2D圆形障碍物-黑色
        for i = 1:size(obstacles.circles, 1)
            circle = obstacles.circles(i, :);
            rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                                  2*circle(3), 2*circle(3)], ...
                     'Curvature', [1 1], ...
                     'FaceColor', [0 0 0 0], ...
                     'EdgeColor', 'none');
        end
        plot(config.startPoint(1), config.startPoint(2), 'go', ...
             'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
        plot(config.goalPoint(1), config.goalPoint(2), 'rs', ...
             'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
    end
    
    if result.success
        % 绘制路径
        path = result.path;
        if is3D && size(path,2) >= 3
            plot3(path(:,1), path(:,2), path(:,3), ...
                  'Color', alg.color, ...
                  'LineStyle', alg.linestyle, ...
                  'LineWidth', alg.linewidth);
        else
            plot(path(:,1), path(:,2), ...
                 'Color', alg.color, ...
                 'LineStyle', alg.linestyle, ...
                 'LineWidth', alg.linewidth);
        end
        
        % 在标题中添加性能指标
        title_str = sprintf('%s\nTime:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f', ...
                           alg.name, result.time, result.length, result.nodes, result.smoothness);
        title(title_str, 'FontSize', 13, 'FontWeight', 'bold');
    else
        % 失败情况：只显示算法名称
        title(sprintf('%s\n✗ 规划失败', alg.name), ...
              'FontSize', 13, 'FontWeight', 'bold', 'Color', 'r');
    end
    
    set(gca, 'FontSize', 12);
end

fprintf('✓ 图2: 算法对比结果已显示\n\n');

%% =================== 保存图片 ===================
fprintf('========================================\n');
fprintf('保存论文图片...\n');
fprintf('========================================\n\n');

% 创建保存目录
save_dir = 'paper_figures';
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% 生成文件名（带时间戳）
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% 保存图1（障碍物环境）
fig1_name = sprintf('environment_%s', timestamp);
saveas(fig1, fullfile(save_dir, [fig1_name '.png']));
print(fig1, fullfile(save_dir, [fig1_name '.eps']), '-depsc', '-r300');
fprintf('✓ 图1已保存: %s.png / .eps\n', fig1_name);

% 保存图2（算法对比）
fig2_name = sprintf('algorithm_comparison_%s', timestamp);
saveas(fig2, fullfile(save_dir, [fig2_name '.png']));
print(fig2, fullfile(save_dir, [fig2_name '.eps']), '-depsc', '-r300');
fprintf('✓ 图2已保存: %s.png / .eps\n', fig2_name);

%% =================== 生成性能统计表 ===================
fprintf('\n========================================\n');
fprintf('性能统计表 (LaTeX格式)\n');
fprintf('========================================\n\n');

fprintf('\\begin{table}[htbp]\n');
fprintf('\\centering\n');
fprintf('\\caption{单次运行对比结果}\n');
fprintf('\\label{tab:single_run_comparison}\n');
fprintf('\\begin{tabular}{lcccc}\n');
fprintf('\\hline\n');
fprintf('算法 & 时间(s) & 路径长度(mm) & 节点数 & 平滑度 \\\\\n');
fprintf('\\hline\n');

for i = 1:num_algorithms
    if results{i}.success
        fprintf('%s & %.4f & %.2f & %d & %.4f \\\\\n', ...
                algorithms{i}.name, ...
                results{i}.time, ...
                results{i}.length, ...
                results{i}.nodes, ...
                results{i}.smoothness);
    else
        fprintf('%s & \\multicolumn{4}{c}{失败} \\\\\n', algorithms{i}.name);
    end
end

fprintf('\\hline\n');
fprintf('\\end{tabular}\n');
fprintf('\\end{table}\n\n');

fprintf('========================================\n');
fprintf('实验完成!\n');
fprintf('========================================\n');
fprintf('图1: 障碍物环境\n');
fprintf('图2: 算法对比结果 (5个算法 + 性能柱状图)\n');
fprintf('保存位置: %s\\\n', save_dir);
fprintf('========================================\n\n');

%% =================== 辅助函数 ===================

function smoothness = calculateSmoothness(path)
    % 计算路径平滑度（转角变化率，支持2D/3D）
    if size(path, 1) < 3
        smoothness = 0;
        return;
    end
    
    angles = [];
    for i = 2:size(path,1)-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        n1 = norm(v1); n2 = norm(v2);
        if n1 < eps || n2 < eps, continue; end
        cosA = dot(v1,v2) / (n1*n2);
        cosA = max(-1, min(1, cosA));
        angle = acos(cosA);
        angles(end+1) = angle; %#ok<AGROW>
    end
    
    if ~isempty(angles)
        smoothness = var(angles);
    else
        smoothness = 0;
    end
end

function length = calculatePathLength(path)
    % 计算路径总长度
    if size(path, 1) < 2
        length = 0;
        return;
    end
    
    length = 0;
    for i = 2:size(path, 1)
        length = length + norm(path(i,:) - path(i-1,:));
    end
end
