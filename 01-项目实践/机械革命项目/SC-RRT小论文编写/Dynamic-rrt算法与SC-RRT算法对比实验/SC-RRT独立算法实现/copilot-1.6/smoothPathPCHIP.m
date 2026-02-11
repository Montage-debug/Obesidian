function p = smoothPathPCHIP(p, numPoints)
    % PCHIP平滑路径，采用三次Hermite插值
    if size(p,1)<10, return; end
    idx = 1:max(1,floor(size(p,1)/20)):size(p,1);
    if idx(end) ~= size(p,1)
        idx = [idx, size(p,1)];
    end
    pp = p(idx,:);
    if size(pp,1) < 3
        pp = p;
    end
    t_old = 1:size(pp,1);
    t_new = linspace(1, size(pp,1), numPoints);
    if size(p,2)==2
        x_new = interp1(t_old, pp(:,1), t_new, 'pchip');
        y_new = interp1(t_old, pp(:,2), t_new, 'pchip');
        p = [x_new(:), y_new(:)];
    else
        x_new = interp1(t_old, pp(:,1), t_new, 'pchip');
        y_new = interp1(t_old, pp(:,2), t_new, 'pchip');
        z_new = interp1(t_old, pp(:,3), t_new, 'pchip');
        p = [x_new(:), y_new(:), z_new(:)];
    end
end