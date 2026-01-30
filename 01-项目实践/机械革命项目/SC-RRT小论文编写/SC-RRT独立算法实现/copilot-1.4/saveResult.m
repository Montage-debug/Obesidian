function saveResult(algorithm, config, dimension, env_idx, path, tree, elapsedTime, obstacles, fig, gif_path)
% SAVERESULT �����㷨ִ�н��
%
% �������
%   algorithm    - �㷨����
%   config       - ���ò����ṹ��
%   dimension    - ά�� ('2D' �� '3D')
%   env_idx      - ��������
%   path         - ·������
%   tree         - ���ṹ���ݣ��ڵ�����˫����cell���飩
%   elapsedTime  - ִ��ʱ��
%   obstacles    - �ϰ�����Ϣ����
%   fig          - ͼ�ξ��
%   gif_path     - GIF����·��

    % ��ȡ��ǰʱ���
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % �����������Ŀ¼
    save_dir = fullfile(pwd, 'results');
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    
    % ���ɽ���ļ���
    result_filename = sprintf('%s_result_%s_env%d_%s.mat', ...
        algorithm, dimension, env_idx, timestamp);
    result_path = fullfile(save_dir, result_filename);
    
    % ׼���������ṹ��
    result.algorithm = algorithm;
    result.config = config;
    result.dimension = dimension;
    result.path = path;
    result.tree = tree;
    result.elapsedTime = elapsedTime;
    result.obstacles = obstacles;
    result.timestamp = timestamp;
    
    % ����MAT�ļ�
    save(result_path, 'result');
    fprintf('结果已保存: %s\n', result_filename);
    
    % 保存图片
    if nargin >= 9 && ~isempty(fig)
        % 检查图形句柄是否有效
        if ishandle(fig) && isvalid(fig)
            try
                fig_filename = sprintf('%s_figure_%s_env%d_%s.png', ...
                    algorithm, dimension, env_idx, timestamp);
                fig_path = fullfile(save_dir, fig_filename);
                
                % 确保图形完全渲染
                figure(fig);
                drawnow;
                
                % 保存为高质量PNG
                print(fig, fig_path, '-dpng', '-r300');
                fprintf('图片已保存: %s\n', fig_filename);
            catch ME
                warning('图片保存失败: %s', ME.message);
            end
        else
            warning('图形句柄无效，跳过图片保存');
        end
    end
    
    % ����GIF�����ļ�������ṩ��
    if nargin >= 10 && ~isempty(gif_path) && exist(gif_path, 'file')
        gif_filename = sprintf('%s_animation_%s_env%d_%s.gif', ...
            algorithm, dimension, env_idx, timestamp);
        gif_dest = fullfile(save_dir, gif_filename);
        copyfile(gif_path, gif_dest);
        fprintf('�����ѱ���: %s\n', gif_filename);
    end
end