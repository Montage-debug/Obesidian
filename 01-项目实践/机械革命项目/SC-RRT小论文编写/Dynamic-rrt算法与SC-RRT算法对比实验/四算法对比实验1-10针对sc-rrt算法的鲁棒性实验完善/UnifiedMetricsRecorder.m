classdef UnifiedMetricsRecorder < handle
    % UnifiedMetricsRecorder - 统一性能度量数据记录器
    %
    % 用于论文性能对比实验的统一数据记录系统
    % 
    % 核心度量标准：
    %   1. 收敛时间 (convergence_time): 算法获得首个可行解所需的计算时长
    %   2. 路径代价 (path_cost): 从起始状态到目标状态的累计代价（欧氏距离之和）
    %   3. 计算时间 (planning_time): 算法总运行时间
    %   4. 成功率、树节点数、平滑度等其他指标
    %
    % 使用示例：
    %   recorder = UnifiedMetricsRecorder('2D');
    %   recorder.recordAlgorithmRun('RRT_Basic', env, path, tree, success, time_elapsed);
    %   recorder.recordAlgorithmRun('SC_RRT', env, path, tree, success, time_elapsed, metrics);
    %   recorder.exportToExcel('results/performance_data.xlsx');
    %   recorder.exportToCSV('results/performance_data.csv');
    %
    % 作者: 研究团队
    % 日期: 2025-12-13
    
    properties
        dimension          % '2D' 或 '3D'
        records            % 所有记录的cell数组
        algorithms         % 算法名称列表
        timestamp          % 创建时间戳
        environment_config % 环境配置信息
    end
    
    methods
        function obj = UnifiedMetricsRecorder(dimension, varargin)
            % 构造函数
            % 输入:
            %   dimension - '2D' 或 '3D'
            %   varargin - 可选环境配置参数
            
            obj.dimension = dimension;
            obj.records = {};
            obj.algorithms = {};
            obj.timestamp = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
            
            if nargin > 1
                obj.environment_config = varargin{1};
            else
                obj.environment_config = struct();
            end
        end
        
        function recordAlgorithmRun(obj, algorithm_name, env, path, tree, success, time_elapsed, metrics)
            % 记录单次算法运行结果
            %
            % 输入:
            %   algorithm_name - 算法名称
            %   env - 环境结构体
            %   path - 规划路径
            %   tree - 树结构体
            %   success - 是否成功
            %   time_elapsed - 运行时间（从外部tic/toc测量）
            %   metrics - (可选) SC-RRT等算法的metrics结构体
            
            record = struct();
            record.algorithm = algorithm_name;
            record.timestamp = datetime('now');
            record.success = success;
            record.run_id = length(obj.records) + 1;
            
            % === 核心度量1: 收敛时间 ===
            % 定义：算法获得首个可行解所需的计算时长
            if nargin >= 8 && isfield(metrics, 'convergence_time')
                % SC-RRT提供了详细的convergence_time
                record.convergence_time = metrics.convergence_time;
            elseif isfield(tree, 'convergence_time')
                % 如果树结构中有convergence_time
                record.convergence_time = tree.convergence_time;
            else
                % 对于基础算法，首次找到解的时间即为总时间
                if success
                    record.convergence_time = time_elapsed;
                else
                    record.convergence_time = inf;
                end
            end
            
            % === 核心度量2: 路径代价 ===
            % 定义：从起始状态到目标状态的累计代价（路径长度）
            if nargin >= 8 && isfield(metrics, 'path_length')
                % SC-RRT提供了path_length
                record.path_cost = metrics.path_length;
            elseif isfield(tree, 'path_cost')
                record.path_cost = tree.path_cost;
            elseif isfield(tree, 'path_length')
                record.path_cost = tree.path_length;
            elseif isfield(tree, 'final_cost')
                record.path_cost = tree.final_cost;
            else
                % 计算路径累计代价
                if success && ~isempty(path) && size(path, 1) > 1
                    record.path_cost = sum(vecnorm(diff(path), 2, 2));
                else
                    record.path_cost = inf;
                end
            end
            
            % === 核心度量3: 总计算时间 ===
            if nargin >= 8 && isfield(metrics, 'planning_time')
                record.planning_time = metrics.planning_time;
            elseif isfield(tree, 'planning_time')
                record.planning_time = tree.planning_time;
            else
                record.planning_time = time_elapsed;
            end
            
            % === 其他性能指标 ===
            % 迭代次数
            if nargin >= 8 && isfield(metrics, 'iterations')
                record.iterations = metrics.iterations;
            else
                record.iterations = NaN;
            end
            
            % 树节点数
            if nargin >= 8 && isfield(metrics, 'tree_nodes')
                record.tree_nodes = metrics.tree_nodes;
            elseif isfield(tree, 'vertices')
                record.tree_nodes = size(tree.vertices, 1);
            elseif isfield(tree, 'nodes')
                record.tree_nodes = size(tree.nodes, 1);
            else
                record.tree_nodes = NaN;
            end
            
            % 平滑度
            if nargin >= 8 && isfield(metrics, 'smoothness')
                record.smoothness = metrics.smoothness;
            elseif success && ~isempty(path) && size(path, 1) >= 3
                record.smoothness = obj.calculatePathSmoothness(path);
            else
                record.smoothness = NaN;
            end
            
            % 安全间隙
            if nargin >= 8 && isfield(metrics, 'clearance')
                record.clearance = metrics.clearance;
            else
                record.clearance = NaN;
            end
            
            % 环境信息
            if isfield(env, 'num')
                record.num_obstacles = env.num;
            elseif isfield(env, 'num_obstacles')
                record.num_obstacles = env.num_obstacles;
            else
                record.num_obstacles = NaN;
            end
            
            % 存储记录
            obj.records{end+1} = record;
            
            % 更新算法列表
            if ~ismember(algorithm_name, obj.algorithms)
                obj.algorithms{end+1} = algorithm_name;
            end
        end
        
        function stats = computeStatistics(obj)
            % 计算所有算法的统计数据
            %
            % 输出:
            %   stats - 统计结构体，包含每个算法的统计数据
            
            stats = struct();
            
            for i = 1:length(obj.algorithms)
                algo = obj.algorithms{i};
                algo_records = obj.getAlgorithmRecords(algo);
                
                if isempty(algo_records)
                    continue;
                end
                
                % 成功率
                success_count = sum([algo_records.success]);
                total_count = length(algo_records);
                stats.(algo).success_rate = success_count / total_count * 100;
                stats.(algo).success_count = success_count;
                stats.(algo).total_runs = total_count;
                
                % 收敛时间统计（仅成功的情况）
                conv_times = [algo_records.convergence_time];
                valid_conv = conv_times(isfinite(conv_times));
                if ~isempty(valid_conv)
                    stats.(algo).convergence_time_mean = mean(valid_conv);
                    stats.(algo).convergence_time_std = std(valid_conv);
                    stats.(algo).convergence_time_min = min(valid_conv);
                    stats.(algo).convergence_time_max = max(valid_conv);
                else
                    stats.(algo).convergence_time_mean = NaN;
                    stats.(algo).convergence_time_std = NaN;
                    stats.(algo).convergence_time_min = NaN;
                    stats.(algo).convergence_time_max = NaN;
                end
                
                % 路径代价统计（仅成功的情况）
                path_costs = [algo_records.path_cost];
                valid_costs = path_costs(isfinite(path_costs));
                if ~isempty(valid_costs)
                    stats.(algo).path_cost_mean = mean(valid_costs);
                    stats.(algo).path_cost_std = std(valid_costs);
                    stats.(algo).path_cost_min = min(valid_costs);
                    stats.(algo).path_cost_max = max(valid_costs);
                else
                    stats.(algo).path_cost_mean = NaN;
                    stats.(algo).path_cost_std = NaN;
                    stats.(algo).path_cost_min = NaN;
                    stats.(algo).path_cost_max = NaN;
                end
                
                % 总计算时间统计
                plan_times = [algo_records.planning_time];
                valid_times = plan_times(~isnan(plan_times));
                if ~isempty(valid_times)
                    stats.(algo).planning_time_mean = mean(valid_times);
                    stats.(algo).planning_time_std = std(valid_times);
                else
                    stats.(algo).planning_time_mean = NaN;
                    stats.(algo).planning_time_std = NaN;
                end
                
                % 树节点数统计
                nodes = [algo_records.tree_nodes];
                valid_nodes = nodes(~isnan(nodes));
                if ~isempty(valid_nodes)
                    stats.(algo).tree_nodes_mean = mean(valid_nodes);
                    stats.(algo).tree_nodes_std = std(valid_nodes);
                else
                    stats.(algo).tree_nodes_mean = NaN;
                    stats.(algo).tree_nodes_std = NaN;
                end
                
                % 平滑度统计
                smooth = [algo_records.smoothness];
                valid_smooth = smooth(~isnan(smooth));
                if ~isempty(valid_smooth)
                    stats.(algo).smoothness_mean = mean(valid_smooth);
                    stats.(algo).smoothness_std = std(valid_smooth);
                else
                    stats.(algo).smoothness_mean = NaN;
                    stats.(algo).smoothness_std = NaN;
                end
            end
        end
        
        function algo_records = getAlgorithmRecords(obj, algorithm_name)
            % 获取特定算法的所有记录
            algo_records = [];
            for i = 1:length(obj.records)
                if strcmp(obj.records{i}.algorithm, algorithm_name)
                    algo_records = [algo_records, obj.records{i}];
                end
            end
        end
        
        function exportToCSV(obj, filename)
            % 导出到CSV文件（适合论文表格）
            
            stats = obj.computeStatistics();
            
            % 写入CSV
            fid = fopen(filename, 'w', 'n', 'UTF-8');
            
            % === 文件头信息 ===
            fprintf(fid, '# 性能对比数据汇总 (论文格式)\n');
            fprintf(fid, '# 维度: %s | 总记录数: %d | 生成时间: %s\n', obj.dimension, length(obj.records), datestr(now));
            fprintf(fid, '\n');
            
            % === 性能统计表 (论文使用) ===
            fprintf(fid, '# 主要性能指标 (均值±标准差)\n');
            headers = {'算法', '成功率(%)', '收敛时间(s)', '路径代价', '计算时间(s)', '树节点数', '平滑度'};
            fprintf(fid, '%s\n', strjoin(headers, ','));
            
            rows = {};
            for i = 1:length(obj.algorithms)
                algo = obj.algorithms{i};
                if ~isfield(stats, algo)
                    continue;
                end
                s = stats.(algo);
                
                row = {
                    algo, ...
                    sprintf('%.1f', s.success_rate), ...
                    sprintf('%.3f±%.3f', s.convergence_time_mean, s.convergence_time_std), ...
                    sprintf('%.2f±%.2f', s.path_cost_mean, s.path_cost_std), ...
                    sprintf('%.3f±%.3f', s.planning_time_mean, s.planning_time_std), ...
                    sprintf('%.0f±%.0f', s.tree_nodes_mean, s.tree_nodes_std), ...
                    sprintf('%.3f±%.3f', s.smoothness_mean, s.smoothness_std)
                };
                rows{end+1} = row;
            end
            
            for i = 1:length(rows)
                fprintf(fid, '%s\n', strjoin(rows{i}, ','));
            end
            
            % === 详细数据表 (原始数据) ===
            fprintf(fid, '\n# 详细运行数据\n');
            fprintf(fid, '算法,运行序号,成功,收敛时间(s),路径代价,计算时间(s),树节点数,平滑度\n');
            
            for i = 1:length(obj.records)
                rec = obj.records{i};  % records是cell数组
                fprintf(fid, '%s,%d,%d,%.3f,%.2f,%.3f,%d,%.3f\n', ...
                    rec.algorithm, i, rec.success, ...
                    rec.convergence_time, rec.path_cost, rec.planning_time, ...
                    rec.tree_nodes, rec.smoothness);
            end
            
            fclose(fid);
            fprintf('✓ 已导出CSV到: %s\n', filename);
        end
        
        function exportToMAT(obj, filename)
            % 导出到MAT文件（完整数据）
            
            records = obj.records;
            algorithms = obj.algorithms;
            statistics = obj.computeStatistics();
            dimension = obj.dimension;
            timestamp = obj.timestamp;
            environment_config = obj.environment_config;
            
            save(filename, 'records', 'algorithms', 'statistics', ...
                 'dimension', 'timestamp', 'environment_config');
            
            fprintf('✓ 已导出MAT到: %s\n', filename);
        end
        
        function printSummary(obj)
            % 打印统计摘要
            
            stats = obj.computeStatistics();
            
            fprintf('\n');
            fprintf('═══════════════════════════════════════════════════════════════\n');
            fprintf('                      性能对比统计摘要\n');
            fprintf('═══════════════════════════════════════════════════════════════\n');
            fprintf('维度: %s | 记录数: %d | 算法数: %d\n', ...
                    obj.dimension, length(obj.records), length(obj.algorithms));
            fprintf('───────────────────────────────────────────────────────────────\n');
            fprintf('%-15s %8s %15s %15s %12s\n', ...
                    '算法', '成功率', '收敛时间(s)', '路径代价', '树节点数');
            fprintf('───────────────────────────────────────────────────────────────\n');
            
            for i = 1:length(obj.algorithms)
                algo = obj.algorithms{i};
                if ~isfield(stats, algo)
                    continue;
                end
                s = stats.(algo);
                
                fprintf('%-15s %7.1f%% %7.3f±%.3f %8.2f±%.2f %6.0f±%.0f\n', ...
                        algo, ...
                        s.success_rate, ...
                        s.convergence_time_mean, s.convergence_time_std, ...
                        s.path_cost_mean, s.path_cost_std, ...
                        s.tree_nodes_mean, s.tree_nodes_std);
            end
            fprintf('═══════════════════════════════════════════════════════════════\n\n');
        end
        
        function smoothness = calculatePathSmoothness(~, path)
            % 计算路径平滑度（角度变化的标准差）
            if size(path, 1) < 3
                smoothness = NaN;
                return;
            end
            
            angles = zeros(size(path, 1) - 2, 1);
            for i = 2:size(path, 1)-1
                v1 = path(i, :) - path(i-1, :);
                v2 = path(i+1, :) - path(i, :);
                cos_angle = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-10);
                cos_angle = max(-1, min(1, cos_angle));
                angles(i-1) = acos(cos_angle);
            end
            smoothness = std(angles);
        end
    end
end
