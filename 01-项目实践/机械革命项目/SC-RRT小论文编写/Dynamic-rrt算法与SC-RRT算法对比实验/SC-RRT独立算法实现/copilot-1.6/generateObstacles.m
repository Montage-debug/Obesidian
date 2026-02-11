function obs = generateObstacles(dim,bounds,N,R,start,goal)
    if strcmp(dim,'2D')
        obs.circles = zeros(N,3);
        
        % 分配障碍物到不同区域以增加复杂度
        % 30% 聚簇区域, 25% 走廊/墙壁, 20% 密集区, 25% 随机分布
        nCluster = floor(N * 0.30);
        nCorridor = floor(N * 0.25);
        nDense = floor(N * 0.20);
        nRandom = N - nCluster - nCorridor - nDense;
        
        idx = 1;
        
        % 1. 创建2-3个障碍物聚簇区域（模拟房间或密集障碍区）
        numClusters = 2 + randi(2);
        obsPerCluster = floor(nCluster / numClusters);
        for cluster = 1:numClusters
            % 随机聚簇中心，避开起点终点
            centerOk = false;
            while ~centerOk
                clusterCenter = bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2);
                centerOk = norm(clusterCenter-start)>150 && norm(clusterCenter-goal)>150;
            end
            clusterRadius = 80 + rand*70; % 聚簇半径
            
            for j = 1:obsPerCluster
                ok = false; attempts = 0;
                while ~ok && attempts < 100
                    attempts = attempts + 1;
                    % 在聚簇中心附近生成
                    angle = rand*2*pi;
                    r = rand*clusterRadius;
                    c = clusterCenter + r*[cos(angle), sin(angle)];
                    
                    ok = all(c > bounds(1:2:3)+R & c < bounds(2:2:4)-R) && ...
                         norm(c-start)>2*R && norm(c-goal)>2*R;
                    if ~ok, continue; end
                    if idx>1 && any(sqrt(sum((obs.circles(1:idx-1,1:2)-repmat(c,idx-1,1)).^2,2))<2*R+8)
                        ok=false;
                    end
                end
                if ok
                    obs.circles(idx,:) = [c R];
                    idx = idx + 1;
                end
            end
        end
        
        % 2. 创建走廊/墙壁结构（线性排列）
        numWalls = 2 + randi(2);
        obsPerWall = floor(nCorridor / numWalls);
        for wall = 1:numWalls
            % 随机墙的起点和终点
            wallStart = bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2);
            wallEnd = bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2);
            
            for j = 1:obsPerWall
                ok = false; attempts = 0;
                while ~ok && attempts < 100
                    attempts = attempts + 1;
                    t = (j-1)/(obsPerWall-1);
                    c = wallStart + t*(wallEnd-wallStart);
                    % 添加一些随机偏移
                    c = c + (rand(1,2)-0.5)*20;
                    
                    ok = all(c > bounds(1:2:3)+R & c < bounds(2:2:4)-R) && ...
                         norm(c-start)>2.5*R && norm(c-goal)>2.5*R;
                    if ~ok, continue; end
                    if idx>1 && any(sqrt(sum((obs.circles(1:idx-1,1:2)-repmat(c,idx-1,1)).^2,2))<2*R+8)
                        ok=false;
                    end
                end
                if ok
                    obs.circles(idx,:) = [c R];
                    idx = idx + 1;
                end
            end
        end
        
        % 3. 创建高密度区域
        numDenseAreas = 2;
        obsPerDense = floor(nDense / numDenseAreas);
        for area = 1:numDenseAreas
            denseOk = false;
            while ~denseOk
                denseCenter = bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2);
                denseOk = norm(denseCenter-start)>120 && norm(denseCenter-goal)>120;
            end
            denseRadius = 60 + rand*40;
            
            for j = 1:obsPerDense
                ok = false; attempts = 0;
                while ~ok && attempts < 100
                    attempts = attempts + 1;
                    angle = rand*2*pi;
                    r = rand*denseRadius;
                    c = denseCenter + r*[cos(angle), sin(angle)];
                    
                    ok = all(c > bounds(1:2:3)+R & c < bounds(2:2:4)-R) && ...
                         norm(c-start)>2*R && norm(c-goal)>2*R;
                    if ~ok, continue; end
                    % 密集区域允许更近的距离
                    if idx>1 && any(sqrt(sum((obs.circles(1:idx-1,1:2)-repmat(c,idx-1,1)).^2,2))<2*R+5)
                        ok=false;
                    end
                end
                if ok
                    obs.circles(idx,:) = [c R];
                    idx = idx + 1;
                end
            end
        end
        
        % 4. 剩余随机分布
        for i = idx:N
            ok = false; attempts = 0;
            while ~ok && attempts < 100
                attempts = attempts + 1;
                c = bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2);
                ok = all(c > bounds(1:2:3)+R & c < bounds(2:2:4)-R) && ...
                     norm(c-start)>2*R && norm(c-goal)>2*R;
                if ~ok, continue; end
                if i>1 && any(sqrt(sum((obs.circles(1:i-1,1:2)-repmat(c,i-1,1)).^2,2))<2*R+10)
                    ok=false;
                end
            end
            if ok
                obs.circles(i,:) = [c R];
            end
        end
        
    else
        % 3D障碍物生成 - 增加复杂度
        obs.spheres = zeros(N,4);
        
        % 分配: 40% 聚簇, 25% 层状, 20% 柱状, 15% 随机
        nCluster = floor(N * 0.40);
        nLayer = floor(N * 0.25);
        nColumn = floor(N * 0.20);
        nRandom = N - nCluster - nLayer - nColumn;
        
        idx = 1;
        
        % 1. 创建聚簇区域（3D球形聚簇） - 增强密度
        numClusters = 5 + randi(3); % 5-7个聚簇
        obsPerCluster = floor(nCluster / numClusters);
        for cluster = 1:numClusters
            centerOk = false;
            while ~centerOk
                clusterCenter = bounds(1:2:5) + (bounds(2:2:6)-bounds(1:2:5)).*rand(1,3);
                centerOk = norm(clusterCenter-start)>120 && norm(clusterCenter-goal)>120;
            end
            clusterRadius = 100 + rand*100; % 增大聚簇半径
            
            for j = 1:obsPerCluster
                ok = false; attempts = 0;
                while ~ok && attempts < 100
                    attempts = attempts + 1;
                    theta = rand*2*pi;
                    phi = acos(2*rand-1);
                    r = rand*clusterRadius;
                    c = clusterCenter + r*[sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi)];
                    
                    ok = all(c > bounds(1:2:5)+R & c < bounds(2:2:6)-R) && ...
                         norm(c-start)>2*R && norm(c-goal)>2*R;
                    if ~ok, continue; end
                    if idx>1 && any(sqrt(sum((obs.spheres(1:idx-1,1:3)-repmat(c,idx-1,1)).^2,2))<2*R+8)
                        ok=false;
                    end
                end
                if ok
                    obs.spheres(idx,:) = [c R];
                    idx = idx + 1;
                end
            end
        end
        
        % 2. 层状结构（不同高度的障碍物平面） - 增加层数
        numLayers = 5; % 增加到5层
        obsPerLayer = floor(nLayer / numLayers);
        for layer = 1:numLayers
            zHeight = bounds(5) + (bounds(6)-bounds(5))*(layer/(numLayers+1));
            
            for j = 1:obsPerLayer
                ok = false; attempts = 0;
                while ~ok && attempts < 100
                    attempts = attempts + 1;
                    c = [bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2), zHeight + (rand-0.5)*50];
                    
                    ok = all(c > bounds(1:2:5)+R & c < bounds(2:2:6)-R) && ...
                         norm(c-start)>2*R && norm(c-goal)>2*R;
                    if ~ok, continue; end
                    if idx>1 && any(sqrt(sum((obs.spheres(1:idx-1,1:3)-repmat(c,idx-1,1)).^2,2))<2*R+8)
                        ok=false;
                    end
                end
                if ok
                    obs.spheres(idx,:) = [c R];
                    idx = idx + 1;
                end
            end
        end
        
        % 3. 柱状结构（垂直排列） - 增加柱数
        numColumns = 6; % 增加到6根柱
        obsPerColumn = floor(nColumn / numColumns);
        for col = 1:numColumns
            colBase = bounds(1:2:3) + (bounds(2:2:4)-bounds(1:2:3)).*rand(1,2);
            
            for j = 1:obsPerColumn
                ok = false; attempts = 0;
                while ~ok && attempts < 100
                    attempts = attempts + 1;
                    zPos = bounds(5) + (bounds(6)-bounds(5))*rand;
                    c = [colBase + (rand(1,2)-0.5)*30, zPos];
                    
                    ok = all(c > bounds(1:2:5)+R & c < bounds(2:2:6)-R) && ...
                         norm(c-start)>2.5*R && norm(c-goal)>2.5*R;
                    if ~ok, continue; end
                    if idx>1 && any(sqrt(sum((obs.spheres(1:idx-1,1:3)-repmat(c,idx-1,1)).^2,2))<2*R+8)
                        ok=false;
                    end
                end
                if ok
                    obs.spheres(idx,:) = [c R];
                    idx = idx + 1;
                end
            end
        end
        
        % 4. 剩余随机分布
        for i = idx:N
            ok = false; attempts = 0;
            while ~ok && attempts < 100
                attempts = attempts + 1;
                c = bounds(1:2:5) + (bounds(2:2:6)-bounds(1:2:5)).*rand(1,3);
                ok = all(c > bounds(1:2:5)+R & c < bounds(2:2:6)-R) && ...
                     norm(c-start)>2*R && norm(c-goal)>2*R;
                if ~ok, continue; end
                if i>1 && any(sqrt(sum((obs.spheres(1:i-1,1:3)-repmat(c,i-1,1)).^2,2))<2*R+10)
                    ok=false;
                end
            end
            if ok
                obs.spheres(i,:) = [c R];
            end
        end
    end
end
