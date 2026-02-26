function obs = generateObstacles(dim, bounds, N, R, start, goal)
% generateObstacles - 生成恰好 N 个不重叠、大小随机的障碍物
%
%   R 作为基准半径，每个障碍物的实际半径在 [0.4R, 2.0R] 间随机
%   2D: obs.circles  = [N×3]  每行 [cx, cy, ri]
%   3D: obs.spheres  = [N×4]  每行 [cx, cy, cz, ri]
%
%   保证:
%     1. 障碍物数量恰好 = N
%     2. 障碍物不与起点/终点重叠（安全距离 = ri + 2*R）
%     3. 障碍物之间不重叠（中心距 > ri + rj）
%     4. 障碍物完全在边界内（边距 = ri）

    Rmin = 0.4 * R;    % 最小半径
    Rmax = 2.0 * R;    % 最大半径
    safeMargin = 2 * R;  % 起终点额外安全余量

    if strcmp(dim, '2D')
        % ---------- 2D ----------
        % 边界用最大半径留余量（候选点生成范围），精确检查在内部
        loMax = [bounds(1) + Rmax,  bounds(3) + Rmax];
        hiMax = [bounds(2) - Rmax,  bounds(4) - Rmax];
        spanMax = hiMax - loMax;

        placed = zeros(N, 3);   % [cx cy ri]
        count  = 0;

        while count < N
            batchSize = min((N - count) * 8, 8000);
            candidates = loMax + rand(batchSize, 2) .* spanMax;
            radii      = Rmin + rand(batchSize, 1) * (Rmax - Rmin);

            for k = 1:batchSize
                if count >= N, break; end
                c  = candidates(k, :);
                ri = radii(k);

                % 1. 边界检查（圆完全在内）
                if c(1)-ri < bounds(1) || c(1)+ri > bounds(2), continue; end
                if c(2)-ri < bounds(3) || c(2)+ri > bounds(4), continue; end

                % 2. 与起点终点的安全距离
                if norm(c - start(1:2)) < ri + safeMargin, continue; end
                if norm(c - goal(1:2))  < ri + safeMargin, continue; end

                % 3. 与已放置障碍物不重叠（中心距 > ri + rj）
                if count > 0
                    dists = sqrt(sum((placed(1:count,1:2) - c).^2, 2));
                    if any(dists < ri + placed(1:count, 3)), continue; end
                end

                count = count + 1;
                placed(count, :) = [c, ri];
            end
        end

        obs.circles = placed;
        fprintf('    [generateObstacles] 2D: %d 个, 半径范围 [%.1f, %.1f]\n', ...
                N, min(placed(:,3)), max(placed(:,3)));

    else
        % ---------- 3D ----------
        loMax = [bounds(1)+Rmax, bounds(3)+Rmax, bounds(5)+Rmax];
        hiMax = [bounds(2)-Rmax, bounds(4)-Rmax, bounds(6)-Rmax];
        spanMax = hiMax - loMax;

        placed = zeros(N, 4);   % [cx cy cz ri]
        count  = 0;

        while count < N
            batchSize = min((N - count) * 8, 8000);
            candidates = loMax + rand(batchSize, 3) .* spanMax;
            radii      = Rmin + rand(batchSize, 1) * (Rmax - Rmin);

            for k = 1:batchSize
                if count >= N, break; end
                c  = candidates(k, :);
                ri = radii(k);

                if c(1)-ri < bounds(1) || c(1)+ri > bounds(2), continue; end
                if c(2)-ri < bounds(3) || c(2)+ri > bounds(4), continue; end
                if c(3)-ri < bounds(5) || c(3)+ri > bounds(6), continue; end

                if norm(c - start(1:3)) < ri + safeMargin, continue; end
                if norm(c - goal(1:3))  < ri + safeMargin, continue; end

                if count > 0
                    dists = sqrt(sum((placed(1:count,1:3) - c).^2, 2));
                    if any(dists < ri + placed(1:count, 4)), continue; end
                end

                count = count + 1;
                placed(count, :) = [c, ri];
            end
        end

        obs.spheres = placed;
        fprintf('    [generateObstacles] 3D: %d 个, 半径范围 [%.1f, %.1f]\n', ...
                N, min(placed(:,4)), max(placed(:,4)));
    end
end
