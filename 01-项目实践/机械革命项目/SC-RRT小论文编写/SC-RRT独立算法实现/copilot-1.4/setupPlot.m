function setupPlot(fig,dim,bounds,start,goal,obs)
    clf(fig); 
    hold on;
    
    if strcmp(dim,'2D')
        axis equal; axis(bounds);
        xlabel('X'); ylabel('Y');
        plot(start(1),start(2),'go','MarkerSize',10,'MarkerFaceColor','g');
        plot(goal(1),goal(2),'rs','MarkerSize',10,'MarkerFaceColor','r');
        
        % 绘制障碍物为黑色实心圆形
        for i = 1:size(obs.circles,1)
            theta = linspace(0, 2*pi, 50);
            x = obs.circles(i,1) + obs.circles(i,3) * cos(theta);
            y = obs.circles(i,2) + obs.circles(i,3) * sin(theta);
            fill(x, y, 'k', 'EdgeColor', 'k', 'FaceAlpha', 1);
        end
    else
        view(3); axis equal; axis(bounds);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        scatter3(start(1),start(2),start(3),60,'g','filled');
        scatter3(goal(1),goal(2),goal(3),60,'r','filled');
        
        % 绘制3D障碍物为黑色实心球体
        [X,Y,Z] = sphere(20);  % 增加到20以提高球体精细度
        for k = 1:size(obs.spheres,1)
            surf(X*obs.spheres(k,4)+obs.spheres(k,1),...
                 Y*obs.spheres(k,4)+obs.spheres(k,2),...
                 Z*obs.spheres(k,4)+obs.spheres(k,3),...
                 'FaceAlpha',1,'EdgeColor','none','FaceColor','k');
        end
    end
    
    grid on;
    box on;
    
    % ����ͼ������
    set(gca, 'SortMethod', 'childorder');
    drawnow limitrate;
end