classdef PerformanceCollector < handle
    % PerformanceCollector - 算法性能数据收集和管理系统
    %
    % 功能特点:
    %   1. 收集多个算法的性能指标
    %   2. 支持多次运行的统计分析
    %   3. 导出Excel格式的对比报告
    %   4. 保存MAT格式的原始数据
    %   5. 自动生成性能对比图表
    %
    % 使用示例:
    %   collector = PerformanceCollector();
    %   collector.addResult('RRT_Basic', env, path, tree, success, metrics);
    %   collector.addResult('SC-RRT', env, path, tree, success, metrics);
    %   collector.exportToExcel('results/performance_comparison.xlsx');
    %   collector.saveToMAT('results/performance_data.mat');
    %   collector.plotComparison('results/comparison_charts.png');
    %
    % 作者: GitHub Copilot
    % 日期: 2025-12-11
    
    properties
        results       % 存储所有算法结果的cell数组
        algorithms    % 算法名称列表
        timestamp     % 创建时间戳
    end
    
    methods
        function obj = PerformanceCollector()
            % 构造函数
            obj.results = {};
            obj.algorithms = {};
            obj.timestamp = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
        end
        
        function addResult(obj, algorithm_name, env, path, tree, success, metrics)
            % 添加一次算法运行结果
            %
            % 输入:
            %   algorithm_name - 算法名称 (字符串)
            %   env           - 环境结构体
            %   path          - 规划路径
            %   tree          - 树结构体
            %   success       - 是否成功
            %   metrics       - 性能指标结构体
            
            result = struct();
            result.algorithm = algorithm_name;
            result.timestamp = datetime('now');
            result.dimension = env.dimension;
            result.num_obstacles = env.num;
            result.success = success;
            
            % 提取性能指标
            if isfield(metrics, 'iterations')
                result.iterations = metrics.iterations;
            else
                result.iterations = NaN;
            end
            
            if isfield(metrics, 'tree_nodes')
                result.tree_nodes = metrics.tree_nodes;
            else
                result.tree_nodes = size(tree.nodes, 1);
            end
            
            if isfield(metrics, 'path_length')
                result.path_length = metrics.path_length;
            else
                if success && ~isempty(path)
                    result.path_length = obj.computePathLength(path);
                else
                    result.path_length = inf;
                end
            end
            
            if isfield(metrics, 'planning_time')
                result.planning_time = metrics.planning_time;
            else
                result.planning_time = NaN;
            end
            
            if isfield(metrics, 'smoothness')
                result.smoothness = metrics.smoothness;
            else
                result.smoothness = NaN;
            end
            
            if isfield(metrics, 'clearance')
                result.clearance = metrics.clearance;
            else
                result.clearance = NaN;
            end
            
            % 新增指标: 路径效率
            if isfield(metrics, 'path_efficiency')
                result.path_efficiency = metrics.path_efficiency;
            else
                result.path_efficiency = NaN;
            end
            
            % 新增指标: 节点利用率
            if isfield(metrics, 'node_efficiency')
                result.node_efficiency = metrics.node_efficiency;
            else
                result.node_efficiency = NaN;
            end
            
            % 新增指标: 平均转角
            if isfield(metrics, 'avg_turning_angle')
                result.avg_turning_angle = metrics.avg_turning_angle;
            else
                result.avg_turning_angle = NaN;
            end
            
            % 新增指标: 最大转角
            if isfield(metrics, 'max_turning_angle')
                result.max_turning_angle = metrics.max_turning_angle;
            else
                result.max_turning_angle = NaN;
            end
            
            % 新增指标: 路径密度
            if isfield(metrics, 'path_density')
                result.path_density = metrics.path_density;
            else
                result.path_density = NaN;
            end
            
            % 新增指标: 最小安全间隙
            if isfield(metrics, 'min_clearance')
                result.min_clearance = metrics.min_clearance;
            else
                result.min_clearance = NaN;
            end
            
            % 新增指标: 平均安全间隙
            if isfield(metrics, 'avg_clearance')
                result.avg_clearance = metrics.avg_clearance;
            else
                result.avg_clearance = NaN;
            end
            
            % 存储结果
            obj.results{end+1} = result;
            
            % 更新算法列表
            if ~ismember(algorithm_name, obj.algorithms)
                obj.algorithms{end+1} = algorithm_name;
            end
            
            fprintf('>> 已添加结果: %s [%s, 障碍物:%d, 成功:%d, 时间:%.3fs]\n', ...
                algorithm_name, result.dimension, result.num_obstacles, ...
                result.success, result.planning_time);
        end
        
        function summary = getSummary(obj)
            % 获取所有算法的统计摘要
            %
            % 返回:
            %   summary - 包含每个算法统计信息的结构体数组
            
            summary = struct();
            
            for i = 1:length(obj.algorithms)
                algo = obj.algorithms{i};
                
                % 筛选该算法的所有结果
                algo_results = obj.filterResults('algorithm', algo);
                
                if isempty(algo_results)
                    continue;
                end
                
                % 计算统计量
                n = length(algo_results);
                success_count = sum(cellfun(@(x) x.success, algo_results));
                
                % 只对成功的结果计算路径相关指标
                success_results = algo_results(cellfun(@(y) y.success, algo_results));
                
                % 提取数值数组用于计算均值和标准差
                all_iterations = cellfun(@(x) x.iterations, algo_results);
                all_tree_nodes = cellfun(@(x) x.tree_nodes, algo_results);
                all_planning_time = cellfun(@(x) x.planning_time, algo_results);
                all_smoothness = cellfun(@(x) x.smoothness, algo_results);
                all_clearance = cellfun(@(x) x.clearance, algo_results);
                
                if ~isempty(success_results)
                    succ_path_length = cellfun(@(x) x.path_length, success_results);
                    succ_path_efficiency = cellfun(@(x) x.path_efficiency, success_results);
                    succ_node_efficiency = cellfun(@(x) x.node_efficiency, success_results);
                    succ_avg_turning = cellfun(@(x) x.avg_turning_angle, success_results);
                    succ_max_turning = cellfun(@(x) x.max_turning_angle, success_results);
                    succ_path_density = cellfun(@(x) x.path_density, success_results);
                    succ_min_clearance = cellfun(@(x) x.min_clearance, success_results);
                    succ_avg_clearance = cellfun(@(x) x.avg_clearance, success_results);
                else
                    succ_path_length = NaN;
                    succ_path_efficiency = NaN;
                    succ_node_efficiency = NaN;
                    succ_avg_turning = NaN;
                    succ_max_turning = NaN;
                    succ_path_density = NaN;
                    succ_min_clearance = NaN;
                    succ_avg_clearance = NaN;
                end
                
                summary.(obj.safeFieldName(algo)) = struct(...
                    'algorithm', algo, ...
                    'runs', n, ...
                    'success_count', success_count, ...
                    'success_rate', success_count / n, ...
                    'avg_iterations', mean(all_iterations, 'omitnan'), ...
                    'avg_tree_nodes', mean(all_tree_nodes, 'omitnan'), ...
                    'std_tree_nodes', std(all_tree_nodes, 0, 'omitnan'), ...
                    'avg_path_length', mean(succ_path_length, 'omitnan'), ...
                    'std_path_length', std(succ_path_length, 0, 'omitnan'), ...
                    'avg_planning_time', mean(all_planning_time, 'omitnan'), ...
                    'std_planning_time', std(all_planning_time, 0, 'omitnan'), ...
                    'avg_smoothness', mean(all_smoothness, 'omitnan'), ...
                    'std_smoothness', std(all_smoothness, 0, 'omitnan'), ...
                    'avg_clearance', mean(all_clearance, 'omitnan'), ...
                    'avg_path_efficiency', mean(succ_path_efficiency, 'omitnan'), ...
                    'std_path_efficiency', std(succ_path_efficiency, 0, 'omitnan'), ...
                    'avg_node_efficiency', mean(succ_node_efficiency, 'omitnan'), ...
                    'avg_turning_angle', mean(succ_avg_turning, 'omitnan'), ...
                    'max_turning_angle', max(succ_max_turning), ...
                    'avg_path_density', mean(succ_path_density, 'omitnan'), ...
                    'min_clearance', min(succ_min_clearance), ...
                    'avg_avg_clearance', mean(succ_avg_clearance, 'omitnan') ...
                );
            end
        end
        
        function exportToExcel(obj, filename, varargin)
            % 导出性能数据到Excel文件
            %
            % 输入:
            %   filename - Excel文件路径
            %   varargin - 可选参数:
            %     'IncludeRawData' - 是否包含原始数据 (默认: true)
            
            p = inputParser;
            addRequired(p, 'filename', @ischar);
            addParameter(p, 'IncludeRawData', true, @islogical);
            parse(p, filename, varargin{:});
            
            include_raw = p.Results.IncludeRawData;
            
            % 确保目录存在
            [filepath, ~, ~] = fileparts(filename);
            if ~isempty(filepath) && ~exist(filepath, 'dir')
                mkdir(filepath);
            end
            
            fprintf('正在导出性能数据到Excel: %s\n', filename);
            
            % Sheet 1: 统计摘要
            summary = obj.getSummary();
            summary_table = obj.summaryToTable(summary);
            writetable(summary_table, filename, 'Sheet', '统计摘要');
            
            % Sheet 2: 原始数据 (可选)
            if include_raw && ~isempty(obj.results)
                raw_table = obj.resultsToTable(obj.results);
                writetable(raw_table, filename, 'Sheet', '原始数据');
            end
            
            % Sheet 3: 按算法分组
            for i = 1:length(obj.algorithms)
                algo = obj.algorithms{i};
                algo_results = obj.filterResults('algorithm', algo);
                
                if ~isempty(algo_results)
                    algo_table = obj.resultsToTable(algo_results);
                    sheet_name = obj.safeSheetName(algo);
                    writetable(algo_table, filename, 'Sheet', sheet_name);
                end
            end
            
            fprintf('✓ Excel文件已保存: %s\n', filename);
            fprintf('  包含 %d 个算法, 共 %d 次运行结果\n', ...
                length(obj.algorithms), length(obj.results));
        end
        
        function saveToMAT(obj, filename)
            % 保存完整数据到MAT文件
            %
            % 输入:
            %   filename - MAT文件路径
            
            % 确保目录存在
            [filepath, ~, ~] = fileparts(filename);
            if ~isempty(filepath) && ~exist(filepath, 'dir')
                mkdir(filepath);
            end
            
            % 保存数据
            results = obj.results;
            algorithms = obj.algorithms;
            timestamp = obj.timestamp;
            summary = obj.getSummary();
            
            save(filename, 'results', 'algorithms', 'timestamp', 'summary', '-v7.3');
            
            fprintf('✓ MAT数据文件已保存: %s\n', filename);
        end
        
        function plotComparison(obj, save_path, varargin)
            % 生成性能对比图表
            %
            % 输入:
            %   save_path - 图片保存路径
            %   varargin  - 可选参数:
            %     'Metrics' - 要对比的指标 (默认: 所有)
            
            p = inputParser;
            addRequired(p, 'save_path', @ischar);
            addParameter(p, 'Metrics', {'path_length', 'planning_time', 'tree_nodes', 'success_rate'}, @iscell);
            parse(p, save_path, varargin{:});
            
            metrics_to_plot = p.Results.Metrics;
            
            % 创建图形
            fig = figure('Position', [100, 100, 1200, 800]);
            
            % 获取统计摘要
            summary = obj.getSummary();
            algo_names = fieldnames(summary);
            n_algos = length(algo_names);
            
            if n_algos == 0
                warning('没有数据可绘制');
                return;
            end
            
            % 准备数据
            n_metrics = length(metrics_to_plot);
            for i = 1:n_metrics
                subplot(2, 2, i);
                
                metric = metrics_to_plot{i};
                values = zeros(n_algos, 1);
                
                for j = 1:n_algos
                    algo = algo_names{j};
                    field_name = ['avg_' metric];
                    
                    if strcmp(metric, 'success_rate')
                        field_name = 'success_rate';
                    end
                    
                    if isfield(summary.(algo), field_name)
                        values(j) = summary.(algo).(field_name);
                    else
                        values(j) = NaN;
                    end
                end
                
                % 绘制柱状图
                bar(values);
                set(gca, 'XTickLabel', cellfun(@(x) summary.(x).algorithm, algo_names, 'UniformOutput', false));
                xtickangle(45);
                
                % 设置标题和标签
                switch metric
                    case 'path_length'
                        title('平均路径长度');
                        ylabel('长度 (m)');
                    case 'planning_time'
                        title('平均规划时间');
                        ylabel('时间 (s)');
                    case 'tree_nodes'
                        title('平均树节点数');
                        ylabel('节点数');
                    case 'success_rate'
                        title('成功率');
                        ylabel('成功率 (%)');
                        ylim([0, 1]);
                    case 'smoothness'
                        title('平均平滑度');
                        ylabel('平滑度指标');
                    case 'clearance'
                        title('平均间隙');
                        ylabel('间隙 (m)');
                    otherwise
                        title(metric);
                end
                
                grid on;
            end
            
            % 添加总标题
            sgtitle(sprintf('算法性能对比 (生成时间: %s)', char(obj.timestamp)));
            
            % 保存图片
            [filepath, ~, ~] = fileparts(save_path);
            if ~isempty(filepath) && ~exist(filepath, 'dir')
                mkdir(filepath);
            end
            
            saveas(fig, save_path);
            saveas(fig, strrep(save_path, '.png', '_hires.png'), 'png');
            set(gcf, 'PaperPositionMode', 'auto');
            print(strrep(save_path, '.png', '_hires.png'), '-dpng', '-r300');
            
            fprintf('✓ 对比图表已保存: %s\n', save_path);
            
            close(fig);
        end
        
        function printSummary(obj)
            % 在命令行打印统计摘要
            
            fprintf('\n');
            fprintf('╔════════════════════════════════════════════════════════════╗\n');
            fprintf('║              算法性能对比统计摘要                          ║\n');
            fprintf('╚════════════════════════════════════════════════════════════╝\n');
            fprintf('生成时间: %s\n', char(obj.timestamp));
            fprintf('总运行次数: %d\n', length(obj.results));
            fprintf('算法数量: %d\n\n', length(obj.algorithms));
            
            summary = obj.getSummary();
            algo_names = fieldnames(summary);
            
            for i = 1:length(algo_names)
                algo = algo_names{i};
                s = summary.(algo);
                
                fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
                fprintf('算法: %s\n', s.algorithm);
                fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
                fprintf('  运行次数:     %d (成功: %d)\n', s.runs, s.success_count);
                fprintf('  成功率:       %.1f%%\n', s.success_rate * 100);
                fprintf('  路径长度:     %.2f ± %.2f mm\n', s.avg_path_length, s.std_path_length);
                fprintf('  规划时间:     %.4f ± %.4f s\n', s.avg_planning_time, s.std_planning_time);
                fprintf('  平滑度:       %.4f ± %.4f rad\n', s.avg_smoothness, s.std_smoothness);
                fprintf('  路径效率:     %.4f ± %.4f\n', s.avg_path_efficiency, s.std_path_efficiency);
                fprintf('  树节点数:     %.1f ± %.1f\n', s.avg_tree_nodes, s.std_tree_nodes);
                fprintf('  最小安全间隙: %.2f mm\n', s.min_clearance);
                fprintf('  平均转角:     %.2f°\n', s.avg_turning_angle);
                fprintf('\n');
            end
            
            fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
            
            % ===== SCI论文格式对比表 =====
            fprintf('╔════════════════════════════════════════════════════════════════════════════════════════════════╗\n');
            fprintf('║                      SCI论文格式 - 算法性能对比表 (Mean ± Std)                                ║\n');
            fprintf('╚════════════════════════════════════════════════════════════════════════════════════════════════╝\n');
            fprintf('%-22s | %-22s | %-18s | %-14s | %-14s | %-12s\n', ...
                'Algorithm', 'Path Length (mm)', 'Time (s)', 'Smoothness', 'Efficiency', 'Success(%)');
            fprintf('─────────────────────────────────────────────────────────────────────────────────────────────────\n');
            for i = 1:length(algo_names)
                s = summary.(algo_names{i});
                fprintf('%-22s | %8.2f ± %7.2f | %7.4f ± %6.4f | %6.4f ± %4.4f | %6.4f ± %4.4f | %6.1f\n', ...
                    s.algorithm, ...
                    s.avg_path_length, s.std_path_length, ...
                    s.avg_planning_time, s.std_planning_time, ...
                    s.avg_smoothness, s.std_smoothness, ...
                    s.avg_path_efficiency, s.std_path_efficiency, ...
                    s.success_rate * 100);
            end
            fprintf('─────────────────────────────────────────────────────────────────────────────────────────────────\n');
            fprintf('注: Smoothness = 角度变化标准差(rad), 越低越好 | Efficiency = 直线距离/路径长度, 越高越好\n\n');
        end
    end
    
    methods (Access = private)
        function filtered = filterResults(obj, field, value)
            % 根据字段值筛选结果
            filtered = {};
            for i = 1:length(obj.results)
                if isfield(obj.results{i}, field) && strcmp(obj.results{i}.(field), value)
                    filtered{end+1} = obj.results{i};
                end
            end
        end
        
        function len = computePathLength(~, path)
            % 计算路径长度
            len = 0;
            for i = 1:size(path, 1)-1
                len = len + norm(path(i+1, :) - path(i, :));
            end
        end
        
        function tbl = summaryToTable(~, summary)
            % 将统计摘要转换为表格
            algo_names = fieldnames(summary);
            n = length(algo_names);
            
            algorithms = cell(n, 1);
            runs = zeros(n, 1);
            success_rate = zeros(n, 1);
            avg_iterations = zeros(n, 1);
            avg_tree_nodes = zeros(n, 1);
            avg_path_length = zeros(n, 1);
            avg_planning_time = zeros(n, 1);
            avg_smoothness = zeros(n, 1);
            avg_clearance = zeros(n, 1);
            avg_path_efficiency = zeros(n, 1);
            avg_node_efficiency = zeros(n, 1);
            avg_turning_angle = zeros(n, 1);
            max_turning_angle = zeros(n, 1);
            avg_path_density = zeros(n, 1);
            min_clearance = zeros(n, 1);
            avg_avg_clearance = zeros(n, 1);
            
            for i = 1:n
                s = summary.(algo_names{i});
                algorithms{i} = s.algorithm;
                runs(i) = s.runs;
                success_rate(i) = s.success_rate;
                avg_iterations(i) = s.avg_iterations;
                avg_tree_nodes(i) = s.avg_tree_nodes;
                avg_path_length(i) = s.avg_path_length;
                avg_planning_time(i) = s.avg_planning_time;
                avg_smoothness(i) = s.avg_smoothness;
                avg_clearance(i) = s.avg_clearance;
                avg_path_efficiency(i) = s.avg_path_efficiency;
                avg_node_efficiency(i) = s.avg_node_efficiency;
                avg_turning_angle(i) = s.avg_turning_angle;
                max_turning_angle(i) = s.max_turning_angle;
                avg_path_density(i) = s.avg_path_density;
                min_clearance(i) = s.min_clearance;
                avg_avg_clearance(i) = s.avg_avg_clearance;
            end
            
            tbl = table(algorithms, runs, success_rate, avg_iterations, avg_tree_nodes, ...
                avg_path_length, avg_planning_time, avg_smoothness, avg_clearance, ...
                avg_path_efficiency, avg_node_efficiency, avg_turning_angle, max_turning_angle, ...
                avg_path_density, min_clearance, avg_avg_clearance, ...
                'VariableNames', {'算法', '运行次数', '成功率', '平均迭代次数', '平均节点数', ...
                '平均路径长度', '平均规划时间', '平均平滑度', '平均间隙', ...
                '路径效率', '节点利用率', '平均转角', '最大转角', ...
                '路径密度', '最小间隙', '平均平均间隙'});
        end
        
        function tbl = resultsToTable(~, results)
            % 将结果数组转换为表格
            n = length(results);
            
            algorithms = cell(n, 1);
            timestamps = cell(n, 1);
            dimensions = cell(n, 1);  % 改为cell数组以支持字符串
            num_obstacles = zeros(n, 1);
            success = zeros(n, 1);
            iterations = zeros(n, 1);
            tree_nodes = zeros(n, 1);
            path_length = zeros(n, 1);
            planning_time = zeros(n, 1);
            smoothness = zeros(n, 1);
            clearance = zeros(n, 1);
            
            for i = 1:n
                r = results{i};
                algorithms{i} = r.algorithm;
                timestamps{i} = char(r.timestamp);
                
                % 处理dimension字段(可能是字符串或数字)
                if isnumeric(r.dimension)
                    dimensions{i} = sprintf('%dD', r.dimension);
                else
                    dimensions{i} = r.dimension;
                end
                
                num_obstacles(i) = r.num_obstacles;
                success(i) = r.success;
                iterations(i) = r.iterations;
                tree_nodes(i) = r.tree_nodes;
                path_length(i) = r.path_length;
                planning_time(i) = r.planning_time;
                smoothness(i) = r.smoothness;
                clearance(i) = r.clearance;
            end
            
            tbl = table(algorithms, timestamps, dimensions, num_obstacles, success, ...
                iterations, tree_nodes, path_length, planning_time, smoothness, clearance, ...
                'VariableNames', {'算法', '时间戳', '维度', '障碍物数', '成功', ...
                '迭代次数', '节点数', '路径长度', '规划时间', '平滑度', '间隙'});
        end
        
        function safe_name = safeFieldName(~, name)
            % 将字符串转换为安全的字段名
            safe_name = matlab.lang.makeValidName(name);
        end
        
        function safe_name = safeSheetName(~, name)
            % 将字符串转换为安全的Excel工作表名
            safe_name = name;
            % Excel工作表名限制31字符
            if length(safe_name) > 31
                safe_name = safe_name(1:31);
            end
            % 移除非法字符
            safe_name = regexprep(safe_name, '[:\\/\?\*\[\]]', '_');
        end
    end
end
