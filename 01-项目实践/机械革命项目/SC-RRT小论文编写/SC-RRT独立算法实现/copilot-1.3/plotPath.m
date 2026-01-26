function plotPath(fig, dim, p)
    figure(fig); 
    hold on;
    
    if strcmp(dim,'2D')
        % 绘制2D路径
        plot(p(:,1),p(:,2),'b-','LineWidth',3);
        
        % 关键点标注显示能力
        step = max(1, floor(size(p,1)/20));  % 最多显示20关键点
        plot(p(1:step:end,1),p(1:step:end,2),'bo','MarkerSize',6,'MarkerFaceColor','b');
    else
        % 绘制3D路径
        plot3(p(:,1),p(:,2),p(:,3),'b-','LineWidth',3);
        
        % 关键点标注显示能力
        step = max(1, floor(size(p,1)/20));  % 最多显示20关键点
        scatter3(p(1:step:end,1),p(1:step:end,2),p(1:step:end,3),20,'b','filled');
    end
    
    drawnow limitrate;
end