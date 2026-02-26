% =========================================================================
%     椭球体显示功能集成指南
% =========================================================================
% 本文件提供了将椭球体动态显示集成到现有对比实验的方法
% 
% 新增文件:
%   • plotEllipsoid.m - 椭球体/椭圆绘制函数
%   • calculateDualEllipsoidParams.m - 双椭球参数计算
%   • calculatePotentialMeetPoint.m - 交汇点计算
%   • SC_RRT_with_Ellipsoid_Display.m - 完整集成模块
%   • test_ellipsoid_display.m - 测试脚本
%   • INTEGRATION_GUIDE.m - 本文件
% 
% =========================================================================

%% ========== 方法1: 在single_run_comparison.m中添加椭球体显示 ==========

% 在single_run_comparison.m中找到SC-RRT算法的运行部分，修改如下：

% ---- 原代码 ----
% elseif strcmp(alg.type, 'sc_rrt')
%     fig_temp = figure('Visible', 'off');
%     
%     tic;
%     [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...

% ---- 修改为 ----
% elseif strcmp(alg.type, 'sc_rrt')
%     % 检查是否启用椭球体显示
%     enable_ellipsoid = true;  % 设为 false 禁用椭球体显示
%     
%     if enable_ellipsoid
%         % 创建椭球体显示窗口
%         fig_temp = figure('Name', [alg.name ' - 椭球体显示'], ...
%             'Position', [950 100 900 800]);
%         
%         % 初始化椭球体窗口
%         hold on; axis equal; grid on;
%         xlim(config.bounds(1:2));
%         ylim(config.bounds(3:4));
%         xlabel('X (mm)'); ylabel('Y (mm)');
%         title([alg.name ' - 椭球体约束动态显示']);
%         
%         % 绘制障碍物和起终点
%         for i = 1:size(obstacles.circles, 1)
%             circle = obstacles.circles(i, :);
%             rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
%                                   2*circle(3), 2*circle(3)], ...
%                      'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3]);
%         end
%         plot(config.startPoint(1), config.startPoint(2), 'go', 'MarkerSize', 12);
%         plot(config.goalPoint(1), config.goalPoint(2), 'rs', 'MarkerSize', 12);
%     else
%         fig_temp = figure('Visible', 'off');
%     end
%     
%     % 运行SC_RRT_with_Ellipsoid_Display获得椭球体窗口
%     tic;
%     [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...

%% ========== 方法2: 创建独立的椭球体显示脚本 ==========

% 创建 demo_with_ellipsoid.m 文件:

% clear; clc; close all;
% 
% % 配置
% config.bounds = [0 1500 0 1500];
% config.startPoint = [400 400];
% config.goalPoint = [1100 1100];
% config.numObstacles = 225;
% config.seed = 42;
% 
% % 生成环境
% rng(config.seed);
% obstacles = generateObstacles('2D', config.bounds, config.numObstacles, 15, ...
%                               config.startPoint, config.goalPoint);
% 
% % 运行SC-RRT算法（带椭球体显示）
% [tree, path, success, metrics, fig_ellipsoid] = SC_RRT_with_Ellipsoid_Display(...
%     config.startPoint, config.goalPoint, config.bounds, obstacles, ...
%     'MaxIterations', 10000, 'Mode', 'pid');

%% ========== 方法3: 在compare_algorithms.m中添加椭球体显示 ==========

% 在compare_algorithms.m中找到SC-RRT算法运行部分，添加椭球体显示标志:

% 修改 algorithms 结构体定义，添加 show_ellipsoid 字段:

% % SC-RRT 算法配置
% algorithms{5} = struct(...
%     'name', 'SC-RRT Adaptive', ...
%     'type', 'sc_rrt', ...
%     'params', struct('Mode', 'adaptive', 'UseParetoFrontier', true), ...
%     'color', [0.00 0.45 0.74], ...
%     'show_ellipsoid', true);  % 新增: 是否显示椭球体
% 
% % 在运行循环中处理椭球体显示:
% if strcmp(alg.type, 'sc_rrt')
%     if alg.show_ellipsoid
%         fig_ellipsoid = figure('Name', [alg.name ' - 椭球体显示']);
%         % ... 初始化椭球体窗口 ...
%     else
%         fig_ellipsoid = [];
%     end
%     
%     % ... 运行SC_RRT ...

%% ========== 椭球体函数使用示例 ==========

% 示例1: 直接使用plotEllipsoid绘制椭球体
example1_plotEllipsoid = {...
    'figure;',...
    'hold on; axis equal; grid on;',...
    'xlim([0 1500]); ylim([0 1500]);',...
    '',...
    '% 定义参数',...
    'startPoint = [400 400];',...
    'meetPoint = [750 750];',...
    'goalPoint = [1100 1100];',...
    'cBestA = norm(meetPoint - startPoint) * 1.2;',...
    'cBestB = norm(goalPoint - meetPoint) * 1.2;',...
    '',...
    '% 绘制椭球体',...
    'plotEllipsoid(startPoint, meetPoint, cBestA, 2, [0.8 0.3 0.3]);  % 红色',...
    'plotEllipsoid(meetPoint, goalPoint, cBestB, 2, [0.3 0.3 0.8]);   % 蓝色',...
    '',...
    '% 标记焦点和交汇点',...
    'scatter(startPoint(1), startPoint(2), 100, ''g'', ''filled'');',...
    'scatter(meetPoint(1), meetPoint(2), 150, ''y'', ''filled'', ''d'');',...
    'scatter(goalPoint(1), goalPoint(2), 100, ''r'', ''filled'');',...
};

% 示例2: 使用calculateDualEllipsoidParams计算椭球参数
example2_dualParams = {...
    '% 构建树结构 [N×(m+4)] 格式: [坐标, 父节点, 代价, ...]',...
    'treeA = [nodes, ones(size(nodes,1),1), costs, zeros(size(nodes,1),1)];',...
    'treeB = [nodes, ones(size(nodes,1),1), costs, zeros(size(nodes,1),1)];',...
    '',...
    '% 计算椭球参数',...
    '[c_best_A, c_best_B, c_min_A, c_min_B] = calculateDualEllipsoidParams(...',...
    '    treeA, treeB, startPoint, goalPoint, meetPoint, m, 1.2);',...
    '',...
    '% 使用计算结果',...
    'if isfinite(c_best_A) && c_best_A > c_min_A * 1.02',...
    '    plotEllipsoid(startPoint, meetPoint, c_best_A, m, color_A);',...
    'end',...
};

% 示例3: 使用calculatePotentialMeetPoint计算交汇点
example3_meetPoint = {...
    '% 假设treeA, treeB已定义且包含有效的树数据',...
    '[meetPoint, centroidA, centroidB] = calculatePotentialMeetPoint(...',...
    '    treeA, treeB, startPoint, goalPoint, m);',...
    '',...
    '% meetPoint 是动态计算的交汇点',...
    '% 可用于后续的椭球体计算和可视化',...
};

fprintf('\n========================================\n');
fprintf('椭球体显示功能集成指南\n');
fprintf('========================================\n\n');

fprintf('已新增的文件:\n');
fprintf('  1. plotEllipsoid.m - 椭球体/椭圆绘制\n');
fprintf('  2. calculateDualEllipsoidParams.m - 双椭球参数计算\n');
fprintf('  3. calculatePotentialMeetPoint.m - 交汇点计算\n');
fprintf('  4. SC_RRT_with_Ellipsoid_Display.m - 完整集成模块\n');
fprintf('  5. test_ellipsoid_display.m - 测试脚本\n\n');

fprintf('快速开始:\n');
fprintf('  >> test_ellipsoid_display  % 运行椭球体显示测试\n\n');

fprintf('集成方法:\n');
fprintf('  方法1: 修改 single_run_comparison.m\n');
fprintf('  方法2: 创建独立的椭球体显示脚本\n');
fprintf('  方法3: 修改 compare_algorithms.m\n\n');

fprintf('关键函数:\n');
fprintf('  • plotEllipsoid(focus1, focus2, cBest, m, color)\n');
fprintf('  • calculateDualEllipsoidParams(treeA, treeB, ...)\n');
fprintf('  • calculatePotentialMeetPoint(treeA, treeB, ...)\n\n');

fprintf('========================================\n\n');
