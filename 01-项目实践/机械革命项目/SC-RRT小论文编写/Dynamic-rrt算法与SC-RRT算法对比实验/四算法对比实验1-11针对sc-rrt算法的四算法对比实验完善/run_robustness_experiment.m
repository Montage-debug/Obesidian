% =========================================================================
%   SC-RRT 鲁棒性验证实验  ——  SCI 论文级别
%   版本: v1.0    日期: 2026-03-14
% =========================================================================
%
%  【实验设计】四组独立实验，全面验证 SC-RRT 算法的稳定性与鲁棒性：
%
%  ┌──────┬────────────────────────────────────┬────────┬──────────────────┐
%  │ 实验 │ 描述                               │  次数  │ 主要验证目标      │
%  ├──────┼────────────────────────────────────┼────────┼──────────────────┤
%  │  E1  │ SC-RRT 固定2D环境超次数稳定性测试  │ N=2000 │ 重复性+一致性     │
%  │  E2  │ 障碍物密度梯度鲁棒性（4算法对比）  │ 100/级 │ 环境适应性        │
%  │  E3  │ SC-RRT 固定3D环境稳定性测试        │ N=500  │ 三维空间稳定性    │
%  │  E4  │ 多算法随机环境对比鲁棒性           │ N=200  │ 统计显著优越性    │
%  └──────┴────────────────────────────────────┴────────┴──────────────────┘
%
%  【核心统计量】
%   • 成功率 (Success Rate)          • 变异系数 CV = σ/μ×100%
%   • 均值 ± 标准差 (Mean ± Std)     • 95% 置信区间 (CI95)
%   • 中位数 + 四分位距 (IQR)        • Wilcoxon 秩和检验 p 值
%
%  【输出文件】（保存至 robustness_results/ 子目录）
%   Figure 1 : SC-RRT 2D 稳定性 — 四指标箱线图 + 累积成功率曲线
%   Figure 2 : SC-RRT 2D 指标累积分布函数 (CDF)
%   Figure 3 : 成功率 vs 障碍物密度（四算法折线图）
%   Figure 4 : 路径长度 / 规划时间 vs 障碍物密度（误差棒图）
%   Figure 5 : SC-RRT 3D 稳定性 — 四指标箱线图
%   Figure 6 : 多算法鲁棒性对比箱线图（2D 随机环境）
%   Figure 7 : Wilcoxon 检验显著性热图
%   Figure 8 : 滚动均值收敛曲线（稳定性量化）
%   CSV/MAT  : 所有原始数据 + 统计汇总
%   LaTeX    : 可直接粘贴的论文对比表
%
% =========================================================================
clc; clear; close all;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║     SC-RRT 鲁棒性验证实验  (SCI 论文级别)                        ║\n');
fprintf('║     版本: v1.0   日期: 2026-03-14                                ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');

%% =========================================================================
%%   §1  全局配置（用户可按需修改下方参数）
%% =========================================================================

% ── 实验开关 ────────────────────────────────────────────────────────────
RUN_EXP1 = true;    % 实验1: SC-RRT 2D 固定环境  N=2000
RUN_EXP2 = true;    % 实验2: 障碍物密度梯度鲁棒性
RUN_EXP3 = true;    % 实验3: SC-RRT 3D 固定环境  N=500
RUN_EXP4 = true;    % 实验4: 多算法随机环境对比  N=200

% ── 运行次数 ────────────────────────────────────────────────────────────
cfg.exp1_runs  = 2000;  % 实验1 重复次数（验证大样本稳定性的核心）
cfg.exp2_runs  = 100;   % 实验2 每密度级每算法重复次数
cfg.exp3_runs  = 500;   % 实验3 重复次数
cfg.exp4_runs  = 200;   % 实验4 每算法重复次数




% ── 环境参数 ────────────────────────────────────────────────────────────
cfg.bounds_2d    = [0 1500 0 1500];
cfg.bounds_3d    = [0 1500 0 1500 0 1500];
cfg.start_2d     = [400 400];
cfg.goal_2d      = [1100 1100];
cfg.start_3d     = [200 200 200];
cfg.goal_3d      = [1300 1300 1300];
cfg.num_obs_2d   = 225;     % 标准2D障碍物密度
cfg.num_obs_3d   = 400;     % 标准3D障碍物密度
cfg.obs_radius   = 15;      % 球体/圆形障碍物半径
cfg.max_iter_2d  = 5000;    % 2D 最大迭代次数
cfg.max_iter_3d  = 8000;    % 3D 最大迭代次数
cfg.env_seed     = 42;      % 固定环境生成随机种子（保证可复现）

% ── 实验2密度梯度级别 ─────────────────────────────────────────────────
cfg.density_levels = [50, 100, 150, 200, 275, 350];

% ── 对比算法配置 ─────────────────────────────────────────────────────
cfg.compare_algos  = {'SC_RRT_Basic', 'RRT_Star_Basic', 'RRT_Connect_Basic', 'RRT_Basic'};
cfg.algo_labels    = {'SC-RRT', 'RRT*', 'RRT-Connect', 'RRT'};
cfg.algo_colors    = {[0.00 0.45 0.74], [0.47 0.67 0.19], [0.93 0.69 0.13], [0.85 0.33 0.10]};

% ── 输出设置 ─────────────────────────────────────────────────────────
cfg.save_data      = true;
cfg.save_figures   = true;
cfg.fig_format     = 'png';   % 'png' | 'pdf' | 'eps'
cfg.fig_dpi        = 300;
cfg.output_dir     = fullfile(fileparts(mfilename('fullpath')), 'robustness_results');

%% =========================================================================
%%   §2  初始化
%% =========================================================================

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
addpath(script_dir);
addpath(fullfile(script_dir, 'sc_rrt'));
addpath(fullfile(script_dir, 'dynamic_rrt'));
cd(script_dir);

if (cfg.save_data || cfg.save_figures) && ~exist(cfg.output_dir, 'dir')
    mkdir(cfg.output_dir);
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
results   = struct();

direct_dist_2d = norm(cfg.goal_2d - cfg.start_2d);
direct_dist_3d = norm(cfg.goal_3d - cfg.start_3d);

fprintf('工作目录 : %s\n', script_dir);
fprintf('输出目录 : %s\n', cfg.output_dir);
fprintf('时间戳   : %s\n\n', timestamp);
fprintf('实验规划:\n');
fprintf('  E1  SC-RRT 2D固定环境稳定性  : %d 次\n', cfg.exp1_runs);
fprintf('  E2  障碍物密度梯度鲁棒性     : %d 密度级 × %d 算法 × %d 次\n', ...
        length(cfg.density_levels), length(cfg.compare_algos), cfg.exp2_runs);
fprintf('  E3  SC-RRT 3D固定环境稳定性  : %d 次\n', cfg.exp3_runs);
fprintf('  E4  多算法随机环境对比       : %d 算法 × %d 次\n', ...
        length(cfg.compare_algos), cfg.exp4_runs);
fprintf('\n');

%% =========================================================================
%%   §3  实验1 — SC-RRT 2D 固定环境超次数稳定性测试（N=2000）
%% =========================================================================
if RUN_EXP1
    fprintf('════════════════════════════════════════════════════════════════\n');
    fprintf('  实验1: SC-RRT 2D 固定环境稳定性测试  (N = %d)\n', cfg.exp1_runs);
    fprintf('  目标 : 同一障碍物布局下，不同随机过程的重复性验证\n');
    fprintf('════════════════════════════════════════════════════════════════\n\n');

    % 生成固定2D环境（固定种子）
    rng(cfg.env_seed);
    env_opts1              = struct();
    env_opts1.start_point  = cfg.start_2d;
    env_opts1.goal_point   = cfg.goal_2d;
    env_opts1.fixed_radius = cfg.obs_radius;
    env_opts1.min_spacing  = 5.0;
    env_opts1.clearance    = 45.0;
    env1 = EnvironmentConfig.generate2DEnvironment(cfg.bounds_2d, cfg.num_obs_2d, env_opts1);
    fprintf('固定2D环境: %d 个圆形障碍物  起点%s → 终点%s\n\n', ...
            env1.num, mat2str(cfg.start_2d), mat2str(cfg.goal_2d));

    N1 = cfg.exp1_runs;
    e1.success     = false(N1, 1);
    e1.plan_time   = nan(N1, 1);
    e1.path_length = nan(N1, 1);
    e1.smoothness  = nan(N1, 1);
    e1.tree_nodes  = nan(N1, 1);
    e1.iterations  = nan(N1, 1);
    e1.path_eff    = nan(N1, 1);

    t_exp1 = tic;
    for k = 1:N1
        rng(1000 + k);   % 每次独立随机状态 → 模拟独立实验
        try
            [path, ~, success, metrics] = SC_RRT_Basic(env1, cfg.max_iter_2d);
            e1.success(k) = success;
            if success && ~isempty(path) && size(path, 1) > 1
                e1.plan_time(k)   = metrics.planning_time;
                e1.path_length(k) = metrics.path_length;
                e1.tree_nodes(k)  = metrics.tree_nodes;
                e1.iterations(k)  = metrics.iterations;
                e1.smoothness(k)  = local_smoothness(path);
                e1.path_eff(k)    = direct_dist_2d / metrics.path_length;
            end
        catch ME
            e1.success(k) = false;
            % 不打印每次异常，避免刷屏
        end

        if mod(k, 100) == 0 || k == N1
            sr  = sum(e1.success(1:k)) / k * 100;
            ela = toc(t_exp1);
            eta = ela / k * (N1 - k);
            fprintf('  [E1] %4d/%d  成功率 %.1f%%  已用 %.0fs  预计剩余 %.0fs\n', ...
                    k, N1, sr, ela, eta);
        end
    end

    results.exp1       = e1;
    results.exp1.env   = env1;
    fprintf('\n✓ 实验1完成，总耗时 %.1f s，最终成功率 %.2f%%\n\n', ...
            toc(t_exp1), sum(e1.success)/N1*100);
end

%% =========================================================================
%%   §4  实验2 — 障碍物密度梯度鲁棒性（6密度 × 4算法 × 100次）
%% =========================================================================
if RUN_EXP2
    nd = length(cfg.density_levels);
    na = length(cfg.compare_algos);
    N2 = cfg.exp2_runs;

    fprintf('════════════════════════════════════════════════════════════════\n');
    fprintf('  实验2: 障碍物密度梯度鲁棒性  (%d 密度 × %d 算法 × %d 次)\n', nd, na, N2);
    fprintf('  目标 : 验证算法在不同环境复杂度下的成功率与性能一致性\n');
    fprintf('════════════════════════════════════════════════════════════════\n\n');

    e2 = struct();
    for ai = 1:na
        aname = cfg.compare_algos{ai};
        e2.(aname).success     = false(nd, N2);
        e2.(aname).plan_time   = nan(nd, N2);
        e2.(aname).path_length = nan(nd, N2);
        e2.(aname).smoothness  = nan(nd, N2);
        e2.(aname).tree_nodes  = nan(nd, N2);
    end

    t_exp2 = tic;
    for di = 1:nd
        num_obs = cfg.density_levels(di);
        fprintf('── 密度级 %d/%d: %d 个障碍物 ──────────────────────────\n', ...
                di, nd, num_obs);

        % 预生成该密度的 N2 个随机环境（保证不同算法面对相同环境）
        envs2 = cell(N2, 1);
        for k = 1:N2
            rng(2000 + di * 1000 + k);
            envs2{k} = generate2DEnvironment(cfg.bounds_2d, num_obs);
        end

        for ai = 1:na
            aname = cfg.compare_algos{ai};
            succ_cnt = 0;
            fprintf('  %-12s ', cfg.algo_labels{ai});
            for k = 1:N2
                rng(3000 + di * 1000 + ai * 100 + k);
                try
                    [path, ~, success, metrics] = local_run_algo(...
                        aname, envs2{k}, cfg.max_iter_2d);
                    e2.(aname).success(di, k) = success;
                    if success && ~isempty(path) && size(path, 1) > 1
                        e2.(aname).plan_time(di, k)   = metrics.planning_time;
                        e2.(aname).path_length(di, k) = metrics.path_length;
                        e2.(aname).smoothness(di, k)  = local_smoothness(path);
                        e2.(aname).tree_nodes(di, k)  = metrics.tree_nodes;
                        succ_cnt = succ_cnt + 1;
                    end
                catch
                    e2.(aname).success(di, k) = false;
                end
            end
            fprintf('成功率 %5.1f%%\n', succ_cnt / N2 * 100);
        end
        fprintf('\n');
    end

    results.exp2              = e2;
    results.exp2.density_levels = cfg.density_levels;
    fprintf('✓ 实验2完成，总耗时 %.1f s\n\n', toc(t_exp2));
end

%% =========================================================================
%%   §5  实验3 — SC-RRT 3D 固定环境稳定性测试（N=500）
%% =========================================================================
if RUN_EXP3
    fprintf('════════════════════════════════════════════════════════════════\n');
    fprintf('  实验3: SC-RRT 3D 固定环境稳定性测试  (N = %d)\n', cfg.exp3_runs);
    fprintf('  目标 : 在三维空间中验证算法的数值稳定性\n');
    fprintf('════════════════════════════════════════════════════════════════\n\n');

    rng(cfg.env_seed);
    env_opts3              = struct();
    env_opts3.start_point  = cfg.start_3d;
    env_opts3.goal_point   = cfg.goal_3d;
    env_opts3.fixed_radius = cfg.obs_radius * 1.5;  % 3D空间稍大球体
    env_opts3.min_spacing  = 20.0;
    env_opts3.clearance    = 60.0;
    env3 = EnvironmentConfig.generate3DEnvironment(cfg.bounds_3d, cfg.num_obs_3d, env_opts3);
    fprintf('固定3D环境: %d 个球体障碍物  起点%s → 终点%s\n\n', ...
            env3.num, mat2str(cfg.start_3d), mat2str(cfg.goal_3d));

    N3 = cfg.exp3_runs;
    e3.success     = false(N3, 1);
    e3.plan_time   = nan(N3, 1);
    e3.path_length = nan(N3, 1);
    e3.smoothness  = nan(N3, 1);
    e3.tree_nodes  = nan(N3, 1);
    e3.iterations  = nan(N3, 1);

    t_exp3 = tic;
    for k = 1:N3
        rng(5000 + k);
        try
            [path, ~, success, metrics] = SC_RRT_Basic(env3, cfg.max_iter_3d);
            e3.success(k) = success;
            if success && ~isempty(path) && size(path, 1) > 1
                e3.plan_time(k)   = metrics.planning_time;
                e3.path_length(k) = metrics.path_length;
                e3.tree_nodes(k)  = metrics.tree_nodes;
                e3.iterations(k)  = metrics.iterations;
                e3.smoothness(k)  = local_smoothness(path);
            end
        catch
            e3.success(k) = false;
        end

        if mod(k, 50) == 0 || k == N3
            sr  = sum(e3.success(1:k)) / k * 100;
            ela = toc(t_exp3);
            eta = ela / k * (N3 - k);
            fprintf('  [E3] %4d/%d  成功率 %.1f%%  已用 %.0fs  预计剩余 %.0fs\n', ...
                    k, N3, sr, ela, eta);
        end
    end

    results.exp3     = e3;
    results.exp3.env = env3;
    fprintf('\n✓ 实验3完成，总耗时 %.1f s，最终成功率 %.2f%%\n\n', ...
            toc(t_exp3), sum(e3.success)/N3*100);
end

%% =========================================================================
%%   §6  实验4 — 多算法随机环境对比鲁棒性（4算法 × N=200）
%% =========================================================================
if RUN_EXP4
    na = length(cfg.compare_algos);
    N4 = cfg.exp4_runs;

    fprintf('════════════════════════════════════════════════════════════════\n');
    fprintf('  实验4: 多算法对比鲁棒性  (%d 算法 × %d 次随机环境)\n', na, N4);
    fprintf('  目标 : 统计显著性比较 — SC-RRT vs 基线算法\n');
    fprintf('════════════════════════════════════════════════════════════════\n\n');

    e4 = struct();
    for ai = 1:na
        aname = cfg.compare_algos{ai};
        e4.(aname).success     = false(N4, 1);
        e4.(aname).plan_time   = nan(N4, 1);
        e4.(aname).path_length = nan(N4, 1);
        e4.(aname).smoothness  = nan(N4, 1);
        e4.(aname).tree_nodes  = nan(N4, 1);
    end

    % 预生成 N4 个随机2D环境（所有算法面对相同环境集合，保证公平对比）
    fprintf('预生成 %d 个随机2D环境（固定种子，保证每算法面对相同环境集合）...\n', N4);
    envs4 = cell(N4, 1);
    for k = 1:N4
        rng(8000 + k);
        envs4{k} = generate2DEnvironment(cfg.bounds_2d, cfg.num_obs_2d);
    end
    fprintf('✓ 环境生成完成\n\n');

    t_exp4 = tic;
    for ai = 1:na
        aname = cfg.compare_algos{ai};
        fprintf('测试算法 [%d/%d]: %s\n', ai, na, cfg.algo_labels{ai});
        succ_cnt = 0;
        for k = 1:N4
            rng(9000 + ai * 1000 + k);
            try
                [path, ~, success, metrics] = local_run_algo(...
                    aname, envs4{k}, cfg.max_iter_2d);
                e4.(aname).success(k) = success;
                if success && ~isempty(path) && size(path, 1) > 1
                    e4.(aname).plan_time(k)   = metrics.planning_time;
                    e4.(aname).path_length(k) = metrics.path_length;
                    e4.(aname).smoothness(k)  = local_smoothness(path);
                    e4.(aname).tree_nodes(k)  = metrics.tree_nodes;
                    succ_cnt = succ_cnt + 1;
                end
            catch
                e4.(aname).success(k) = false;
            end
            if mod(k, 50) == 0 || k == N4
                fprintf('  进度 %3d/%d  当前成功率 %.1f%%\n', k, N4, succ_cnt/k*100);
            end
        end
        fprintf('\n');
    end

    results.exp4 = e4;
    fprintf('✓ 实验4完成，总耗时 %.1f s\n\n', toc(t_exp4));
end

%% =========================================================================
%%   §7  统计分析（控制台输出）
%% =========================================================================
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('  统计分析结果\n');
fprintf('════════════════════════════════════════════════════════════════\n\n');

if RUN_EXP1
    s1 = local_compute_stats(results.exp1);
    fprintf('【实验1】SC-RRT 2D固定环境稳定性（N=%d）\n', cfg.exp1_runs);
    local_print_stats(s1);
end
if RUN_EXP3
    s3 = local_compute_stats(results.exp3);
    fprintf('【实验3】SC-RRT 3D固定环境稳定性（N=%d）\n', cfg.exp3_runs);
    local_print_stats(s3);
end
if RUN_EXP4
    fprintf('【实验4】多算法对比统计（N=%d 随机环境）\n', cfg.exp4_runs);
    local_print_comparison(results.exp4, cfg.compare_algos, cfg.algo_labels, cfg.exp4_runs);
end

%% =========================================================================
%%   §8  SCI发表级可视化（8张图）
%% =========================================================================
fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('  生成SCI发表级图表\n');
fprintf('════════════════════════════════════════════════════════════════\n\n');

% 统一字体与分辨率设置
set(0, 'DefaultFigureColor',   'w');
set(0, 'DefaultAxesFontSize',  11);
set(0, 'DefaultAxesFontName',  'Times New Roman');
set(0, 'DefaultLineLineWidth', 1.5);

% ── Figure 1: SC-RRT 2D稳定性 — 四指标箱线图 + 累积成功率 ─────────────
if RUN_EXP1
    idx_ok1 = find(results.exp1.success);
    N_ok1   = length(idx_ok1);
    sr1     = N_ok1 / cfg.exp1_runs * 100;

    fig1 = figure('Name', 'Fig1 SC-RRT 2D Stability', ...
                  'Position', [50 50 1300 820]);

    sp_data = {results.exp1.plan_time(idx_ok1),   '规划时间 (s)'; ...
               results.exp1.path_length(idx_ok1), '路径长度 (mm)'; ...
               results.exp1.smoothness(idx_ok1),  '路径平滑度 (rad)'; ...
               results.exp1.tree_nodes(idx_ok1),  '树节点数'};
    sp_titles = {sprintf('规划时间  (SR=%.1f%%)', sr1), ...
                 '路径长度', '路径平滑度', '树节点数'};

    for si = 1:4
        subplot(2, 3, si);
        local_boxplot(sp_data{si,1}, sp_data{si,2}, cfg.algo_colors{1});
        title(sp_titles{si}, 'FontSize', 11);
    end

    % 子图5: 规划时间 vs 路径长度 散点（验证一致性）
    subplot(2, 3, 5);
    scatter(results.exp1.plan_time(idx_ok1), results.exp1.path_length(idx_ok1), ...
            12, 'filled', 'MarkerFaceColor', cfg.algo_colors{1}, 'MarkerFaceAlpha', 0.3);
    xlabel('规划时间 (s)', 'FontSize', 10);
    ylabel('路径长度 (mm)', 'FontSize', 10);
    title('时间-路径长度 相关性');
    grid on; box on;
    cc = corrcoef(results.exp1.plan_time(idx_ok1), results.exp1.path_length(idx_ok1));
    r_val = cc(1, 2);
    text(0.05, 0.92, sprintf('r = %.3f', r_val), 'Units', 'normalized', ...
         'FontSize', 9, 'Color', [0.3 0.3 0.3]);

    % 子图6: 累积成功率收敛曲线
    subplot(2, 3, 6);
    cumsr = cumsum(results.exp1.success) ./ (1:cfg.exp1_runs)' * 100;
    plot(1:cfg.exp1_runs, cumsr, 'Color', cfg.algo_colors{1}, 'LineWidth', 1.8);
    yline(cumsr(end), '--k', sprintf('最终 %.1f%%', cumsr(end)), ...
          'FontSize', 9, 'LabelHorizontalAlignment', 'left');
    xlabel('累计运行次数', 'FontSize', 10);
    ylabel('累积成功率 (%)', 'FontSize', 10);
    title('成功率收敛曲线');
    ylim([max(0, cumsr(end)-15) 100]); grid on; box on;

    sgtitle(sprintf('图1: SC-RRT 2D固定环境稳定性分析 (N=%d次)', cfg.exp1_runs), ...
            'FontSize', 13, 'FontWeight', 'bold');
    local_save_fig(fig1, cfg, 'Fig1_SC_RRT_2D_Stability', timestamp);
end

% ── Figure 2: SC-RRT 2D — CDF累积分布函数 ────────────────────────────
if RUN_EXP1
    idx_ok1 = find(results.exp1.success);
    fig2 = figure('Name', 'Fig2 SC-RRT 2D CDF', 'Position', [80 80 1200 500]);

    cdf_data  = {results.exp1.plan_time(idx_ok1), ...
                 results.exp1.path_length(idx_ok1), ...
                 results.exp1.smoothness(idx_ok1)};
    cdf_xlbl  = {'规划时间 (s)', '路径长度 (mm)', '路径平滑度 (rad)'};
    cdf_title = {'规划时间 CDF', '路径长度 CDF', '路径平滑度 CDF'};

    for si = 1:3
        subplot(1, 3, si);
        d = cdf_data{si};
        d = d(~isnan(d) & isfinite(d));
        d_sorted = sort(d);
        p_pct    = (1:length(d_sorted)) / length(d_sorted) * 100;
        plot(d_sorted, p_pct, 'Color', cfg.algo_colors{1}, 'LineWidth', 2);
        xlabel(cdf_xlbl{si}, 'FontSize', 10);
        ylabel('累积百分比 (%)', 'FontSize', 10);
        title(cdf_title{si}, 'FontSize', 11);
        ylim([0 100]); grid on; box on;
        % 标注 P25 P50 P75 P95
        for pct = [25 50 75 95]
            xp = quantile(d, pct/100);
            xline(xp, ':', sprintf('P%d', pct), ...
                  'Color', [0.5 0.5 0.5], 'FontSize', 8, ...
                  'HandleVisibility', 'off');
        end
    end

    sgtitle(sprintf('图2: SC-RRT 2D指标累积分布函数 (N=%d)', sum(results.exp1.success)), ...
            'FontSize', 13, 'FontWeight', 'bold');
    local_save_fig(fig2, cfg, 'Fig2_SC_RRT_2D_CDF', timestamp);
end

% ── Figure 3: 成功率 vs 障碍物密度（四算法折线图）────────────────────
if RUN_EXP2
    fig3 = figure('Name', 'Fig3 Success Rate vs Density', 'Position', [110 110 720 520]);
    hold on; grid on; box on;
    for ai = 1:length(cfg.compare_algos)
        aname = cfg.compare_algos{ai};
        sr_vec = mean(results.exp2.(aname).success, 2) * 100;
        plot(cfg.density_levels, sr_vec, '-o', ...
             'Color',           cfg.algo_colors{ai}, ...
             'DisplayName',     cfg.algo_labels{ai}, ...
             'LineWidth',       2.2, ...
             'MarkerSize',      8, ...
             'MarkerFaceColor', cfg.algo_colors{ai});
    end
    hold off;
    xlabel('障碍物数量', 'FontSize', 12);
    ylabel('成功率 (%)', 'FontSize', 12);
    ylim([0 105]);
    legend('Location', 'southwest', 'FontSize', 10, 'Box', 'on');
    title(sprintf('图3: 成功率 vs 障碍物密度  (每密度N=%d次)', cfg.exp2_runs), ...
          'FontSize', 12, 'FontWeight', 'bold');
    local_save_fig(fig3, cfg, 'Fig3_SuccessRate_vs_Density', timestamp);
end

% ── Figure 4: 路径长度/规划时间 vs 密度（误差棒）───────────────────────
if RUN_EXP2
    fig4 = figure('Name', 'Fig4 Performance vs Density', 'Position', [140 140 1200 480]);

    metrics_f4 = {'path_length', 'plan_time'};
    ylbl_f4    = {'路径长度 Mean±Std (mm)', '规划时间 Mean±Std (s)'};

    for mi = 1:2
        subplot(1, 2, mi);
        hold on; grid on; box on;
        for ai = 1:length(cfg.compare_algos)
            aname = cfg.compare_algos{ai};
            dm    = results.exp2.(aname).(metrics_f4{mi});   % [nd × N2]
            mu    = mean(dm, 2, 'omitnan');
            sg    = std(dm, 0, 2, 'omitnan');
            errorbar(cfg.density_levels, mu, sg, '-o', ...
                     'Color',           cfg.algo_colors{ai}, ...
                     'DisplayName',     cfg.algo_labels{ai}, ...
                     'LineWidth',       1.8, ...
                     'MarkerSize',      7, ...
                     'MarkerFaceColor', cfg.algo_colors{ai}, ...
                     'CapSize',         5);
        end
        hold off;
        xlabel('障碍物数量', 'FontSize', 11);
        ylabel(ylbl_f4{mi}, 'FontSize', 11);
        legend('Location', 'northwest', 'FontSize', 9, 'Box', 'on');
        title(ylbl_f4{mi}(1:end-10));   % 去掉单位
    end

    sgtitle('图4: 性能指标 vs 障碍物密度（Mean ± Std误差棒）', ...
            'FontSize', 12, 'FontWeight', 'bold');
    local_save_fig(fig4, cfg, 'Fig4_Performance_vs_Density', timestamp);
end

% ── Figure 5: SC-RRT 3D稳定性箱线图 ──────────────────────────────────
if RUN_EXP3
    idx_ok3 = find(results.exp3.success);
    N_ok3   = length(idx_ok3);
    sr3     = N_ok3 / cfg.exp3_runs * 100;
    c3d     = [0.49 0.18 0.56];   % 紫色

    fig5 = figure('Name', 'Fig5 SC-RRT 3D Stability', 'Position', [170 170 1100 740]);

    sp3_data = {results.exp3.plan_time(idx_ok3),   '规划时间 (s)'; ...
                results.exp3.path_length(idx_ok3), '路径长度 (mm)'; ...
                results.exp3.smoothness(idx_ok3),  '路径平滑度 (rad)'; ...
                results.exp3.tree_nodes(idx_ok3),  '树节点数'};

    for si = 1:4
        subplot(2, 2, si);
        local_boxplot(sp3_data{si,1}, sp3_data{si,2}, c3d);
        if si == 1
            title(sprintf('规划时间  (SR=%.1f%%)', sr3), 'FontSize', 11);
        else
            title(sp3_data{si,2}, 'FontSize', 11);
        end
    end

    sgtitle(sprintf('图5: SC-RRT 3D固定环境稳定性分析 (N=%d)', cfg.exp3_runs), ...
            'FontSize', 13, 'FontWeight', 'bold');
    local_save_fig(fig5, cfg, 'Fig5_SC_RRT_3D_Stability', timestamp);
end

% ── Figure 6: 多算法鲁棒性对比箱线图 ─────────────────────────────────
if RUN_EXP4
    fig6 = figure('Name', 'Fig6 Multi-Algo Robustness', 'Position', [200 200 1300 920]);

    mnames = {'plan_time', 'path_length', 'smoothness', 'tree_nodes'};
    yunits = {'规划时间 (s)', '路径长度 (mm)', '路径平滑度 (rad)', '树节点数'};

    for mi = 1:4
        subplot(2, 2, mi);
        hold on;
        na = length(cfg.compare_algos);
        for ai = 1:na
            aname = cfg.compare_algos{ai};
            ok    = find(results.exp4.(aname).success);
            d     = results.exp4.(aname).(mnames{mi})(ok);
            d     = d(~isnan(d) & isfinite(d));
            if isempty(d), continue; end

            q    = quantile(d, [0.25 0.5 0.75]);
            iqrv = q(3) - q(1);
            wl   = max(min(d), q(1) - 1.5*iqrv);
            wh   = min(max(d), q(3) + 1.5*iqrv);
            c    = cfg.algo_colors{ai};
            bw   = 0.28;

            fill([ai-bw ai+bw ai+bw ai-bw], [q(1) q(1) q(3) q(3)], c, ...
                 'FaceAlpha', 0.35, 'EdgeColor', c, 'LineWidth', 1.5);
            plot([ai-bw ai+bw], [q(2) q(2)], '-', 'Color', c, 'LineWidth', 2.5);
            plot([ai ai], [wl q(1)], '-', 'Color', c, 'LineWidth', 1.5);
            plot([ai ai], [q(3) wh],  '-', 'Color', c, 'LineWidth', 1.5);
            plot([ai-bw/2 ai+bw/2], [wl wl], '-', 'Color', c, 'LineWidth', 1.5);
            plot([ai-bw/2 ai+bw/2], [wh wh], '-', 'Color', c, 'LineWidth', 1.5);
            plot(ai, mean(d), 'd', 'MarkerFaceColor', 'k', ...
                 'MarkerEdgeColor', 'k', 'MarkerSize', 7);

            % 抖动散点（最多200个）
            ns  = min(length(d), 200);
            idx_s = randperm(length(d), ns);
            scatter(ai + (rand(ns,1)-0.5)*0.18, d(idx_s), 7, c, ...
                    'filled', 'MarkerFaceAlpha', 0.25);
        end
        hold off;
        set(gca, 'XTick', 1:na, 'XTickLabel', cfg.algo_labels, ...
            'XTickLabelRotation', 10);
        ylabel(yunits{mi}, 'FontSize', 10);
        title(yunits{mi}, 'FontSize', 11);
        xlim([0.4 na+0.6]); grid on; box on;
    end

    sgtitle(sprintf('图6: 多算法鲁棒性对比 (N=%d随机2D环境)', cfg.exp4_runs), ...
            'FontSize', 13, 'FontWeight', 'bold');
    local_save_fig(fig6, cfg, 'Fig6_MultiAlgo_Robustness', timestamp);
end

% ── Figure 7: Wilcoxon 检验显著性热图 ────────────────────────────────
if RUN_EXP4
    fig7 = figure('Name', 'Fig7 Statistical Significance', 'Position', [230 230 1100 380]);

    test_fields  = {'plan_time', 'path_length', 'smoothness'};
    test_titles  = {'规划时间 显著性', '路径长度 显著性', '路径平滑度 显著性'};
    na = length(cfg.compare_algos);

    for mi = 1:3
        subplot(1, 3, mi);
        pmat = ones(na, na);
        for ai = 1:na
            for bi = 1:na
                if ai == bi, continue; end
                aname = cfg.compare_algos{ai};
                bname = cfg.compare_algos{bi};
                da = results.exp4.(aname).(test_fields{mi});
                db = results.exp4.(bname).(test_fields{mi});
                oka = da(results.exp4.(aname).success & ~isnan(da) & isfinite(da));
                okb = db(results.exp4.(bname).success & ~isnan(db) & isfinite(db));
                if length(oka) > 5 && length(okb) > 5
                    [~, pmat(ai,bi)] = ranksum(oka, okb);
                end
            end
        end

        imagesc(-log10(pmat + 1e-15));
        colormap(gca, flipud(hot));
        cb = colorbar;
        cb.Label.String = '-log_{10}(p)';
        caxis([0 4]);
        set(gca, 'XTick', 1:na, 'XTickLabel', cfg.algo_labels, ...
            'XTickLabelRotation', 20, 'YTick', 1:na, ...
            'YTickLabel', cfg.algo_labels);
        title(test_titles{mi}, 'FontSize', 11);

        % 标注显著性符号
        for ai = 1:na
            for bi = 1:na
                if ai == bi
                    text(bi, ai, '—', 'HorizontalAlignment', 'center', ...
                         'VerticalAlignment', 'middle', 'FontSize', 10, 'Color', [0.4 0.4 0.4]);
                else
                    p = pmat(ai, bi);
                    if p < 0.001,   sym = '***';
                    elseif p < 0.01, sym = '**';
                    elseif p < 0.05, sym = '*';
                    else,            sym = 'ns';
                    end
                    fc = 'w';
                    if p > 0.05, fc = [0.2 0.2 0.2]; end
                    text(bi, ai, sym, 'HorizontalAlignment', 'center', ...
                         'VerticalAlignment', 'middle', 'FontSize', 9, 'Color', fc);
                end
            end
        end
    end

    sgtitle('图7: Wilcoxon 秩和检验显著性矩阵（行优于列）', ...
            'FontSize', 13, 'FontWeight', 'bold');
    local_save_fig(fig7, cfg, 'Fig7_Statistical_Significance', timestamp);
end

% ── Figure 8: 滚动均值收敛曲线（稳定性量化）─────────────────────────
if RUN_EXP1
    fig8 = figure('Name', 'Fig8 Rolling Mean Convergence', 'Position', [260 260 1300 500]);
    win  = 100;   % 滑动窗口

    roll_fields = {'plan_time', 'path_length', 'smoothness'};
    roll_units  = {'规划时间 (s)', '路径长度 (mm)', '路径平滑度 (rad)'};

    for mi = 1:3
        subplot(1, 3, mi);
        d = results.exp1.(roll_fields{mi});
        d(~results.exp1.success) = NaN;
        r_mu = movmean(d, win, 'omitnan');
        r_sd = movstd(d,  win, 'omitnan');
        x    = 1:cfg.exp1_runs;

        hold on;
        fill([x fliplr(x)], [(r_mu+r_sd)' fliplr((r_mu-r_sd)')], ...
             cfg.algo_colors{1}, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
        plot(x, r_mu, 'Color', cfg.algo_colors{1}, 'LineWidth', 2);
        hold off;
        xlabel('累计运行次数', 'FontSize', 10);
        ylabel(roll_units{mi}, 'FontSize', 10);
        title(roll_units{mi}, 'FontSize', 11);
        grid on; box on;

        final_mu = mean(d, 'omitnan');
        final_cv = std(d, 'omitnan') / final_mu * 100;
        yline(final_mu, '--k', sprintf('μ=%.3g', final_mu), ...
              'FontSize', 9, 'LabelHorizontalAlignment', 'left');
        text(0.97, 0.06, sprintf('CV=%.1f%%', final_cv), ...
             'Units', 'normalized', 'HorizontalAlignment', 'right', ...
             'FontSize', 10, 'Color', [0.6 0 0], 'FontWeight', 'bold');
    end

    sgtitle(sprintf('图8: SC-RRT 2D 稳定性收敛分析（滑动窗口 w=%d）', win), ...
            'FontSize', 12, 'FontWeight', 'bold');
    local_save_fig(fig8, cfg, 'Fig8_Rolling_Convergence', timestamp);
end

%% =========================================================================
%%   §9  数据导出（CSV + MAT + LaTeX论文表格）
%% =========================================================================
fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('  数据导出\n');
fprintf('════════════════════════════════════════════════════════════════\n\n');

if cfg.save_data
    % 完整 MAT 文件
    mat_path = fullfile(cfg.output_dir, sprintf('robustness_raw_%s.mat', timestamp));
    save(mat_path, 'results', 'cfg', 'timestamp', '-v7.3');
    fprintf('✓ 原始数据 MAT : %s\n', mat_path);

    % 实验1 逐次 CSV
    if RUN_EXP1
        e = results.exp1;
        T = table((1:cfg.exp1_runs)', e.success, e.plan_time, e.path_length, ...
                  e.smoothness, e.tree_nodes, e.iterations, e.path_eff, ...
                  'VariableNames', {'Run','Success','PlanTime_s','PathLength_mm', ...
                                    'Smoothness_rad','TreeNodes','Iterations','PathEfficiency'});
        fp = fullfile(cfg.output_dir, sprintf('exp1_stability_2D_%s.csv', timestamp));
        writetable(T, fp);
        fprintf('✓ 实验1 CSV     : %s\n', fp);
    end

    % 实验2 密度汇总 CSV
    if RUN_EXP2
        nd = length(cfg.density_levels);
        na = length(cfg.compare_algos);
        rows = cell(nd * na, 9);
        ri   = 1;
        for ai = 1:na
            aname = cfg.compare_algos{ai};
            for di = 1:nd
                sr_v = mean(results.exp2.(aname).success(di,:)) * 100;
                rows(ri,:) = {cfg.algo_labels{ai}, cfg.density_levels(di), sr_v, ...
                    mean(results.exp2.(aname).plan_time(di,:), 'omitnan'), ...
                    std( results.exp2.(aname).plan_time(di,:), 'omitnan'), ...
                    mean(results.exp2.(aname).path_length(di,:), 'omitnan'), ...
                    std( results.exp2.(aname).path_length(di,:), 'omitnan'), ...
                    mean(results.exp2.(aname).smoothness(di,:), 'omitnan'), ...
                    std( results.exp2.(aname).smoothness(di,:), 'omitnan')};
                ri = ri + 1;
            end
        end
        T2 = cell2table(rows, 'VariableNames', ...
            {'Algorithm','NumObs','SR_pct','Time_mean','Time_std', ...
             'Len_mean','Len_std','Smooth_mean','Smooth_std'});
        fp2 = fullfile(cfg.output_dir, sprintf('exp2_density_%s.csv', timestamp));
        writetable(T2, fp2);
        fprintf('✓ 实验2 CSV     : %s\n', fp2);
    end

    % 实验3 逐次 CSV
    if RUN_EXP3
        e = results.exp3;
        T3 = table((1:cfg.exp3_runs)', e.success, e.plan_time, e.path_length, ...
                   e.smoothness, e.tree_nodes, ...
                   'VariableNames', {'Run','Success','PlanTime_s','PathLength_mm', ...
                                     'Smoothness_rad','TreeNodes'});
        fp3 = fullfile(cfg.output_dir, sprintf('exp3_stability_3D_%s.csv', timestamp));
        writetable(T3, fp3);
        fprintf('✓ 实验3 CSV     : %s\n', fp3);
    end

    % 实验4 算法对比汇总 CSV
    if RUN_EXP4
        na   = length(cfg.compare_algos);
        rows4 = cell(na, 11);
        for ai = 1:na
            aname = cfg.compare_algos{ai};
            ok    = find(results.exp4.(aname).success);
            n_ok  = length(ok);
            rows4(ai,:) = {cfg.algo_labels{ai}, length(results.exp4.(aname).success), n_ok, ...
                n_ok/length(results.exp4.(aname).success)*100, ...
                mean(results.exp4.(aname).plan_time(ok), 'omitnan'), ...
                std( results.exp4.(aname).plan_time(ok), 'omitnan'), ...
                mean(results.exp4.(aname).path_length(ok), 'omitnan'), ...
                std( results.exp4.(aname).path_length(ok), 'omitnan'), ...
                mean(results.exp4.(aname).smoothness(ok), 'omitnan'), ...
                std( results.exp4.(aname).smoothness(ok), 'omitnan'), ...
                mean(results.exp4.(aname).tree_nodes(ok), 'omitnan')};
        end
        T4 = cell2table(rows4, 'VariableNames', ...
            {'Algorithm','N_total','N_success','SR_pct', ...
             'Time_mean','Time_std','Len_mean','Len_std', ...
             'Smooth_mean','Smooth_std','Nodes_mean'});
        fp4 = fullfile(cfg.output_dir, sprintf('exp4_comparison_%s.csv', timestamp));
        writetable(T4, fp4);
        fprintf('✓ 实验4 CSV     : %s\n', fp4);
    end

    % LaTeX 论文对比表格
    if RUN_EXP4
        tex_path = fullfile(cfg.output_dir, sprintf('paper_table_%s.tex', timestamp));
        local_export_latex(results.exp4, cfg, tex_path, cfg.exp4_runs);
        fprintf('✓ LaTeX 表格    : %s\n', tex_path);
    end
end

%% =========================================================================
%%   §10  完成汇总
%% =========================================================================
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║                   SC-RRT 鲁棒性实验全部完成！                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n');
fprintf('输出目录: %s\n', cfg.output_dir);

if RUN_EXP1
    sr1_final = sum(results.exp1.success) / cfg.exp1_runs * 100;
    cv_t     = std(results.exp1.plan_time(results.exp1.success), 'omitnan') / ...
               mean(results.exp1.plan_time(results.exp1.success), 'omitnan') * 100;
    cv_l     = std(results.exp1.path_length(results.exp1.success), 'omitnan') / ...
               mean(results.exp1.path_length(results.exp1.success), 'omitnan') * 100;
    fprintf('\n核心鲁棒性指标 (可直接写入论文):\n');
    fprintf('  SC-RRT 2D成功率  : %.2f%%  (N=%d, 固定环境)\n', sr1_final, cfg.exp1_runs);
    fprintf('  规划时间 CV      : %.1f%%  → 数值稳定性\n', cv_t);
    fprintf('  路径长度 CV      : %.1f%%  → 路径质量一致性\n', cv_l);
end
fprintf('\n');

%% =========================================================================
%%   §11  本地辅助函数
%% =========================================================================

% ── 统一算法调用接口 ──────────────────────────────────────────────────
function [path, tree, success, metrics] = local_run_algo(algo_name, env, max_iter)
    path    = [];
    tree    = struct('final_cost', inf);
    success = false;
    metrics = struct('planning_time', nan, 'path_length', nan, ...
                     'tree_nodes', nan, 'iterations', nan);
    t0 = tic;
    switch algo_name
        case 'SC_RRT_Basic'
            [path, tree, success, metrics] = SC_RRT_Basic(env, max_iter);

        case 'RRT_Star_Basic'
            [path, tree, success] = RRT_Star_Basic(env, max_iter);
            metrics.planning_time = toc(t0);
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
                if isfield(tree, 'vertices')
                    metrics.tree_nodes = size(tree.vertices, 1);
                end
            end

        case 'RRT_Connect_Basic'
            [path, tree, success] = RRT_Connect_Basic(env, max_iter, [], []);
            metrics.planning_time = toc(t0);
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
                if isfield(tree, 'vertices')
                    metrics.tree_nodes = size(tree.vertices, 1);
                end
            end

        case 'RRT_Basic'
            [path, tree, success] = RRT_Basic(env, max_iter);
            metrics.planning_time = toc(t0);
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
                if isfield(tree, 'vertices')
                    metrics.tree_nodes = size(tree.vertices, 1);
                end
            end

        case 'Informed_RRT_Star_Basic'
            [path, tree, success] = Informed_RRT_Star_Basic(env, max_iter);
            metrics.planning_time = toc(t0);
            if success && ~isempty(path)
                metrics.path_length = tree.final_cost;
            end

        case 'Dynamic_RRT_Basic'
            [path, tree, success, metrics] = Dynamic_RRT_Basic(env, max_iter);

        otherwise
            error('未知算法: %s', algo_name);
    end

    % 补全 path_length（若算法未直接返回）
    if success && ~isempty(path) && size(path,1) > 1
        if ~isfield(metrics,'path_length') || isnan(metrics.path_length) || isinf(metrics.path_length)
            metrics.path_length = sum(vecnorm(diff(path), 2, 2));
        end
    end
    if ~isfield(metrics, 'planning_time') || isnan(metrics.planning_time)
        metrics.planning_time = toc(t0);
    end
    if ~isfield(metrics, 'tree_nodes')
        metrics.tree_nodes = nan;
    end
    if ~isfield(metrics, 'iterations')
        metrics.iterations = nan;
    end
end

% ── 路径平滑度（角度变化标准差）─────────────────────────────────────
function s = local_smoothness(path)
    s = 0;
    if isempty(path) || size(path,1) < 3, return; end
    diffs   = diff(path);
    seg_len = vecnorm(diffs, 2, 2);
    valid   = seg_len > 1e-8;
    if sum(valid) < 2, return; end
    dirs    = diffs(valid,:) ./ seg_len(valid);
    dots    = sum(dirs(1:end-1,:) .* dirs(2:end,:), 2);
    dots    = max(-1, min(1, dots));
    s       = std(acos(dots));
end

% ── 计算统计量 ────────────────────────────────────────────────────────
function stats = local_compute_stats(e)
    idx_ok = find(e.success);
    stats.n_total     = numel(e.success);
    stats.n_success   = numel(idx_ok);
    stats.success_rate = stats.n_success / stats.n_total * 100;

    for fn = {'plan_time','path_length','smoothness','tree_nodes'}
        f = fn{1};
        if ~isfield(e, f), continue; end
        d = e.(f)(idx_ok);
        d = d(~isnan(d) & isfinite(d));
        if isempty(d)
            stats.(f) = struct('mean',nan,'std',nan,'cv',nan,'ci95',nan,...
                               'median',nan,'iqr',nan,'min',nan,'max',nan);
        else
            mu = mean(d); sg = std(d); n = numel(d);
            stats.(f) = struct('mean',mu,'std',sg,'cv',sg/mu*100, ...
                               'ci95',1.96*sg/sqrt(n),'median',median(d), ...
                               'iqr',iqr(d),'min',min(d),'max',max(d));
        end
    end
end

% ── 打印单算法统计汇总 ────────────────────────────────────────────────
function local_print_stats(s)
    fprintf('  成功率: %.2f%%  (%d / %d)\n', s.success_rate, s.n_success, s.n_total);
    fprintf('  %-20s  %-12s  %-12s  %-9s  %-12s\n', ...
            '指标', '均值', '标准差', 'CV(%)', '95%CI±');
    fprintf('  %s\n', repmat('─', 1, 70));
    fns  = {'plan_time','path_length','smoothness','tree_nodes'};
    lbls = {'规划时间 (s)','路径长度 (mm)','路径平滑度 (rad)','树节点数'};
    for fi = 1:4
        if ~isfield(s, fns{fi}), continue; end
        st = s.(fns{fi});
        fprintf('  %-20s  %-12.5g  %-12.5g  %-9.2f  ±%-10.5g\n', ...
                lbls{fi}, st.mean, st.std, st.cv, st.ci95);
    end
    fprintf('\n');
end

% ── 打印多算法对比汇总 ────────────────────────────────────────────────
function local_print_comparison(e4, algos, labels, n_runs)
    fprintf('  %-16s  %-9s  %-18s  %-18s  %-18s  %-10s\n', ...
            '算法', '成功率(%)', '规划时间 (s)', '路径长度 (mm)', '路径平滑度', '节点数');
    fprintf('  %s\n', repmat('─', 1, 95));
    for ai = 1:numel(algos)
        aname = algos{ai};
        ok = find(e4.(aname).success);
        sr = numel(ok) / n_runs * 100;
        t_str = sprintf('%.3f±%.3f', ...
            mean(e4.(aname).plan_time(ok), 'omitnan'), std(e4.(aname).plan_time(ok), 'omitnan'));
        l_str = sprintf('%.1f±%.1f', ...
            mean(e4.(aname).path_length(ok), 'omitnan'), std(e4.(aname).path_length(ok), 'omitnan'));
        s_str = sprintf('%.4f±%.4f', ...
            mean(e4.(aname).smoothness(ok), 'omitnan'), std(e4.(aname).smoothness(ok), 'omitnan'));
        n_str = sprintf('%.0f', mean(e4.(aname).tree_nodes(ok), 'omitnan'));
        fprintf('  %-16s  %-9.1f  %-18s  %-18s  %-18s  %-10s\n', ...
                labels{ai}, sr, t_str, l_str, s_str, n_str);
    end
    fprintf('\n');
end

% ── 自定义箱线图（带抖动散点）──────────────────────────────────────
function local_boxplot(data, ylbl, rgb)
    data = data(~isnan(data) & isfinite(data));
    if isempty(data)
        text(0.5, 0.5, '无数据', 'HorizontalAlignment', 'center');
        set(gca, 'XTick', []); ylabel(ylbl, 'FontSize', 10); return;
    end
    q    = quantile(data, [0.25 0.5 0.75]);
    iqrv = q(3) - q(1);
    wl   = max(min(data), q(1) - 1.5*iqrv);
    wh   = min(max(data), q(3) + 1.5*iqrv);

    hold on;
    fill([0.65 1.35 1.35 0.65], [q(1) q(1) q(3) q(3)], rgb, ...
         'FaceAlpha', 0.30, 'EdgeColor', rgb, 'LineWidth', 1.8);
    plot([0.65 1.35], [q(2) q(2)], '-', 'Color', rgb, 'LineWidth', 3);
    plot([1 1], [wl q(1)], '-', 'Color', rgb, 'LineWidth', 1.8);
    plot([1 1], [q(3) wh],  '-', 'Color', rgb, 'LineWidth', 1.8);
    plot([0.80 1.20], [wl wl], '-', 'Color', rgb, 'LineWidth', 1.8);
    plot([0.80 1.20], [wh wh], '-', 'Color', rgb, 'LineWidth', 1.8);

    % 离群点
    out_idx = data < wl | data > wh;
    if any(out_idx)
        scatter(ones(sum(out_idx),1), data(out_idx), 18, rgb, ...
                'filled', 'MarkerFaceAlpha', 0.5);
    end
    % 抖动散点（最多800个）
    ns   = min(numel(data), 800);
    sidx = randperm(numel(data), ns);
    scatter(1 + (rand(ns,1)-0.5)*0.28, data(sidx), 6, rgb, ...
            'filled', 'MarkerFaceAlpha', 0.12);
    % 均值菱形
    plot(1, mean(data), 'd', 'MarkerFaceColor', 'k', ...
         'MarkerEdgeColor', 'k', 'MarkerSize', 8);
    hold off;

    xlim([0.4 1.6]); set(gca, 'XTick', []);
    ylabel(ylbl, 'FontSize', 10); grid on; box on;
end

% ── 保存图表（支持 png/pdf/eps）──────────────────────────────────────
function local_save_fig(fig, cfg, name, ts)
    if ~cfg.save_figures, return; end
    fname = fullfile(cfg.output_dir, sprintf('%s_%s.%s', name, ts, cfg.fig_format));
    print(fig, fname, sprintf('-d%s', cfg.fig_format), sprintf('-r%d', cfg.fig_dpi));
    fprintf('✓ 保存图表: [%s]\n', fname);
end

% ── 导出 LaTeX 论文对比表格 ───────────────────────────────────────────
function local_export_latex(e4, cfg, tex_path, n_runs)
    fid = fopen(tex_path, 'w', 'n', 'UTF-8');
    if fid < 0
        warning('无法创建 LaTeX 文件: %s', tex_path);
        return;
    end
    na = numel(cfg.compare_algos);

    % 收集各算法均值（用于高亮最优）
    sr_v = zeros(na,1); t_v = zeros(na,1);
    l_v  = zeros(na,1); s_v = zeros(na,1);
    for ai = 1:na
        aname = cfg.compare_algos{ai};
        ok = find(e4.(aname).success);
        sr_v(ai) = numel(ok) / n_runs * 100;
        t_v(ai)  = mean(e4.(aname).plan_time(ok), 'omitnan');
        l_v(ai)  = mean(e4.(aname).path_length(ok), 'omitnan');
        s_v(ai)  = mean(e4.(aname).smoothness(ok), 'omitnan');
    end

    fprintf(fid, '%% SC-RRT Robustness Experiment - Algorithm Comparison\n');
    fprintf(fid, '%% Generated: %s\n\n', datestr(now));
    fprintf(fid, '\\begin{table}[htbp]\n\\centering\n');
    fprintf(fid, '\\caption{Comparison of path planning algorithms robustness (N=%d random environments)}\n', n_runs);
    fprintf(fid, '\\label{tab:robustness}\n');
    fprintf(fid, '\\begin{tabular}{lcccc}\n\\toprule\n');
    fprintf(fid, 'Algorithm & Success Rate (\\%%) & Planning Time (s) & Path Length (mm) & Smoothness (rad) \\\\\n');
    fprintf(fid, '\\midrule\n');

    for ai = 1:na
        aname = cfg.compare_algos{ai};
        ok = find(e4.(aname).success);
        sr = numel(ok) / n_runs * 100;
        t_mu = mean(e4.(aname).plan_time(ok), 'omitnan');   t_sd = std(e4.(aname).plan_time(ok), 'omitnan');
        l_mu = mean(e4.(aname).path_length(ok), 'omitnan'); l_sd = std(e4.(aname).path_length(ok), 'omitnan');
        s_mu = mean(e4.(aname).smoothness(ok), 'omitnan');  s_sd = std(e4.(aname).smoothness(ok), 'omitnan');

        sr_s = fmt_bold(sprintf('%.1f', sr),          sr == max(sr_v));
        t_s  = fmt_bold(sprintf('%.3f$\\pm$%.3f', t_mu, t_sd), t_mu == min(t_v));
        l_s  = fmt_bold(sprintf('%.1f$\\pm$%.1f', l_mu, l_sd), l_mu == min(l_v));
        s_s  = fmt_bold(sprintf('%.4f$\\pm$%.4f', s_mu, s_sd), s_mu == min(s_v));

        fprintf(fid, '%-14s & %s & %s & %s & %s \\\\\n', ...
                cfg.algo_labels{ai}, sr_s, t_s, l_s, s_s);
    end

    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
    fprintf(fid, '%% Note: Bold = best; $\\pm$ = standard deviation.\n');
    fprintf(fid, '\\end{table}\n');
    fclose(fid);
end

function s = fmt_bold(val_str, is_best)
    if is_best
        s = sprintf('\\textbf{%s}', val_str);
    else
        s = val_str;
    end
end
