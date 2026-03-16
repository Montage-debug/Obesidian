%% 快速验证SC-RRT Anytime优化改进
% 运行3次对比：SC-RRT vs RRT* vs RRT-Connect
clc; clear; close all;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
addpath(fullfile(script_dir, 'sc_rrt'));
addpath(fullfile(script_dir, 'dynamic_rrt'));

NUM_RUNS = 5;
MAX_ITER = 5000;
bounds = [0 1500 0 1500];
num_obstacles = 225;

fprintf('=== SC-RRT Anytime优化验证 ===\n');
fprintf('运行次数: %d, 障碍物: %d\n\n', NUM_RUNS, num_obstacles);

sc_costs = zeros(NUM_RUNS, 1);
sc_times = zeros(NUM_RUNS, 1);
sc_conv = zeros(NUM_RUNS, 1);
sc_smooth = zeros(NUM_RUNS, 1);
sc_improve = zeros(NUM_RUNS, 1);

rrt_star_costs = zeros(NUM_RUNS, 1);
rrt_star_times = zeros(NUM_RUNS, 1);

connect_costs = zeros(NUM_RUNS, 1);
connect_times = zeros(NUM_RUNS, 1);

for run = 1:NUM_RUNS
    fprintf('--- 运行 %d/%d ---\n', run, NUM_RUNS);
    env = generate2DEnvironment(bounds, num_obstacles);
    
    % SC-RRT
    tic;
    [path_sc, tree_sc, success_sc, metrics_sc] = SC_RRT_Basic(env, MAX_ITER);
    t_sc = toc;
    if success_sc
        sc_costs(run) = metrics_sc.path_length;
        sc_times(run) = t_sc;
        sc_conv(run) = metrics_sc.convergence_time;
        sc_smooth(run) = metrics_sc.smoothness;
        sc_improve(run) = metrics_sc.improvement_count;
        fprintf('  SC-RRT: cost=%.2f, conv=%.3fs, total=%.3fs, smooth=%.4f, 改进%d次\n', ...
            metrics_sc.path_length, metrics_sc.convergence_time, t_sc, metrics_sc.smoothness, metrics_sc.improvement_count);
    else
        fprintf('  SC-RRT: 失败\n');
        sc_costs(run) = inf;
    end
    
    % RRT*
    tic;
    [path_rs, tree_rs, success_rs] = RRT_Star_Basic(env, MAX_ITER);
    t_rs = toc;
    if success_rs
        rrt_star_costs(run) = tree_rs.final_cost;
        rrt_star_times(run) = t_rs;
        fprintf('  RRT*:   cost=%.2f, total=%.3fs\n', tree_rs.final_cost, t_rs);
    else
        fprintf('  RRT*: 失败\n');
        rrt_star_costs(run) = inf;
    end
    
    % RRT-Connect
    tic;
    [path_rc, tree_rc, success_rc] = RRT_Connect_Basic(env, MAX_ITER, [], []);
    t_rc = toc;
    if success_rc
        connect_costs(run) = tree_rc.final_cost;
        connect_times(run) = t_rc;
        fprintf('  Connect: cost=%.2f, total=%.3fs\n', tree_rc.final_cost, t_rc);
    else
        fprintf('  Connect: 失败\n');
        connect_costs(run) = inf;
    end
    fprintf('\n');
end

fprintf('\n=== 汇总 (Mean±Std) ===\n');
fprintf('SC-RRT:    cost=%.2f±%.2f, conv=%.3f±%.3fs, total=%.3f±%.3fs, smooth=%.4f, 改进%.1f次\n', ...
    mean(sc_costs), std(sc_costs), mean(sc_conv), std(sc_conv), mean(sc_times), std(sc_times), mean(sc_smooth), mean(sc_improve));
fprintf('RRT*:      cost=%.2f±%.2f, total=%.3f±%.3fs\n', ...
    mean(rrt_star_costs), std(rrt_star_costs), mean(rrt_star_times), std(rrt_star_times));
fprintf('Connect:   cost=%.2f±%.2f, total=%.3f±%.3fs\n', ...
    mean(connect_costs), std(connect_costs), mean(connect_times), std(connect_times));
