function optimal_waypoints = optimize_waypoints(waypoints, init_state)
    eucl_dist = @(curr, target) sqrt((curr(1) - target(1))^2 + (curr(2) - target(2))^2);

    n = size(waypoints, 1);
    optimal_waypoints = zeros(n, 2);
    used = false(n, 1);
    
    current_pose = init_state(:);
    
    for i = 1:n
        nearest_dist = Inf;
        nearest_idx = -1;

        for j = 1:n
            if ~used(j)
                d = eucl_dist(current_pose, waypoints(j, :)');
                if d < nearest_dist
                    nearest_dist = d;
                    nearest_idx = j;
                end
            end
        end

        current_pose = waypoints(nearest_idx, :)';
        used(nearest_idx) = true;
        optimal_waypoints(i, :) = current_pose';
    end
end
