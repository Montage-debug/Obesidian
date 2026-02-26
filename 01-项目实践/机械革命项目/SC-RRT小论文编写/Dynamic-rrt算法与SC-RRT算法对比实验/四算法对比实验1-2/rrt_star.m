function problem = rrt_star(map, max_iter, is_benchmark, rand_seed, variant)
%RRT_STAR -- RRT* is sampling-based algorithm, solves 
% the problem of motion and path planning providing feasible solutions
% taking into account the optimality of a path/motion.
%
% problem = RRT_STAR(map, max_iter, is_benchmark, rand_seed, variant)
% function returns the object of the respective class with the result
%
% 使用方式1 (新的统一环境配置):
%   env = generate2DEnvironment([0 100 0 100], 20);
%   [path, tree, success] = rrt_star(env);  % 使用RRT_Star_Basic
%
% 使用方式2 (原有的类对象方式):
%   problem = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);
%
% map/env       -- 环境结构体或旧格式地图配置
% max_iter      -- number of iteration to solve the problem
% is_benchmark  -- if true saves snapshots of the tree in a special directory
%               boolean variable
% rand_seed     -- a random seed 
% variant       -- what class to choose, class used defines the problem space
% 
%
% for detailed information consult with the help of the _RRT*FN Toolbox_
%
% Olzhas Adiyatov
% 05/15/2013
% Updated: 2025-12-11 - 添加统一环境配置支持

%%% 检测是否使用新的统一环境配置
if nargin >= 1 && isstruct(map)
    % 检查是否是新的环境配置格式
    if isfield(map, 'dimension') && isfield(map, 'obstacles') && ...
       (isfield(map, 'start_point') || isfield(map, 'start'))
        % 新的统一环境配置,调用RRT_Star_Basic
        fprintf('检测到统一环境配置,使用RRT_Star_Basic算法...\n');
        
        % 设置默认参数
        if nargin < 2 || isempty(max_iter)
            max_iter = 5000;
        end
        if nargin < 3 || isempty(is_benchmark)
            step_size = 5;
        else
            step_size = is_benchmark;  % 重用参数位置
        end
        if nargin < 4 || isempty(rand_seed)
            goal_threshold = 5;
        else
            goal_threshold = rand_seed;  % 重用参数位置
        end
        
        % 调用新的RRT_Star_Basic函数
        [path, tree, success] = RRT_Star_Basic(map, max_iter, step_size, goal_threshold);
        
        % 返回结果结构体(保持一致性)
        problem = struct();
        problem.path = path;
        problem.tree = tree;
        problem.success = success;
        problem.env = map;
        return;
    end
end

%%% Configuration block (原有代码)
if nargin < 5
    clear all;
    close all;
    clc;
    
    % load conf
    RAND_SEED   = 5;
    MAX_ITER   = 10e3;
    MAX_NODES    = MAX_ITER;
    
    % here you can specify what class to use, each class represent
    % different model.
    % FNSimple2D provides RRT and RRT* for 2D mobile robot represented as a dot 
    % FNRedundantManipulator represents redundant robotic manipulator, DOF is
    % defined in configuration files.
    

        variant     = 'FNSimple2D';
        MAP = struct('name', 'bench_june1.mat', 'start_point', [-12.5 -5.5], 'goal_point', [7 -3.65]);
%     variant     = 'FNRedundantManipulator';
%     MAP = struct('name', 'bench_redundant_3.mat', 'start_point', [0 0], 'goal_point', [35 35]);
    
    % do we have to benchmark?
    is_benchmark = false;
else
    MAX_NODES   = max_iter;
    MAX_ITER    = max_iter;
    RAND_SEED   = rand_seed;
    MAP = map;
end

addpath(genpath(pwd));

%%% loading configuration file with object model specific options 
if exist(['configure_' variant '.m'], 'file')
    run([pwd '/configure_' variant '.m']);
    CONF = conf;
else
    disp('ERROR: There is no configuration file!')
    return
end

ALGORITHM = 'RRTS';

problem = eval([variant '(RAND_SEED, MAX_NODES, MAP, CONF);']);
%problem = RedundantManipulator(RAND_SEED, MAX_NODES, MAP, CONF);


if (is_benchmark)
    benchmark_record_step = 250;
    benchmark_states = cell(MAX_ITER / benchmark_record_step, 1);
    timestamp = zeros(MAX_ITER / benchmark_record_step, 1);
    iterstamp = zeros(MAX_ITER / benchmark_record_step, 1);
end

%%% Starting a timer

disp(ALGORITHM);
% starting timer
tic;
for ind = 1:MAX_ITER
    new_node = problem.sample();
    nearest_node_ind = problem.nearest(new_node);
    new_node = problem.steer(nearest_node_ind, new_node);   % if new node is very distant from the nearest node we go from the nearest node in the direction of a new node
    if(~problem.obstacle_collision(new_node, nearest_node_ind))
        neighbors = problem.neighbors(new_node, nearest_node_ind);
        min_node_ind = problem.chooseParent(neighbors, nearest_node_ind, new_node);
        new_node_ind = problem.insert_node(min_node_ind, new_node);
        problem.rewire(new_node_ind, neighbors, min_node_ind);
    end
    
    if is_benchmark && (mod(ind, benchmark_record_step) == 0)
        benchmark_states{ind/benchmark_record_step} = problem.copyobj();
        timestamp(ind/benchmark_record_step) = toc;
        iterstamp(ind/benchmark_record_step) = ind;
    end
    
    if(mod(ind, 1000) == 0)
        disp([num2str(ind) ' iterations ' num2str(problem.nodes_added-1) ' nodes in ' num2str(toc) ' rewired ' num2str(problem.num_rewired)]);
    end
end

if (is_benchmark)
    if strcmp(computer, 'GLNXA64');
        result_dir = '/home/olzhas/june_results/';
    else
        result_dir = 'C:\june_results\';
    end
    dir_name = [result_dir datestr(now, 'yyyy-mm-dd')];
    mkdir(dir_name);
    save([dir_name '/' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.mat'], '-v7.3');
    set(gcf, 'Visible', 'off');
    
    % free memory, sometimes there is a memory leak, or matlab decides to
    % free up memory later.
    clear all;
    clear('rrt_star.m');
    
%     problem.plot();
%     saveas(gcf, [dir_name '\' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.fig']);
else
    % Plotting disabled to avoid MATLAB crash with large trees
    % Algorithm completed successfully
end

