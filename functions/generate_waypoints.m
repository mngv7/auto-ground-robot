function waypoints = generate_waypoints(map, x_bounds, y_bounds, no_waypoints, obstacles)
    % GENERATE_WAYPOINTS Generate random valid waypoints
    %
    % map: logical occupancy grid (1 = wall) at 10x resolution
    % x_bounds, y_bounds: [min, max] of world coordinates
    % no_waypoints: number of waypoints to generate
    % obstacles: Nx2 array of obstacle positions in world coordinates
    % Returns: Nx2 array of waypoints

    waypoints = [];

    while size(waypoints, 1) < no_waypoints
        % Generate a random point in world coordinates
        point = [randi(x_bounds), randi(y_bounds)];

        if validate_point(point, x_bounds, y_bounds, map, obstacles)
            waypoints = [waypoints; point];
        end
    end
end

function is_valid = validate_point(point, x_bounds, y_bounds, map, obstacles)
    % VALIDATE_POINT Check if a waypoint is valid
    % point: [x, y] in world coordinates
    % map: logical occupancy grid at 10x resolution (1 = wall)
    % obstacles: Nx2 array in world coordinates
    % Returns: true if valid, false otherwise

    padding = 1;   % safety distance
    is_valid = true;

    x = point(1);
    y = point(2);

    % --- Bounds check ---
    if ~(x_bounds(1) < x && x < x_bounds(2))
        is_valid = false;
        return
    end
    if ~(y_bounds(1) < y && y < y_bounds(2))
        is_valid = false;
        return
    end

    % --- Convert world coordinates to map indices (flip y-axis) ---
    row = round((y_bounds(2) - y) * 10);  % flip y-axis
    col = round(x * 10);

    % Guard against out-of-bounds indexing
    if row <= 0 || col <= 0 || row > size(map,1) || col > size(map,2)
        is_valid = false;
        return
    end

    % --- Direct wall check ---
    if map(row, col) == 1
        is_valid = false;
        return
    end

    % --- Direct obstacle check ---
    if ~isempty(obstacles) && ismember([x, y], obstacles(:,1:2), 'rows')
        is_valid = false;
        return
    end

    % --- Near wall check (4 directions) ---
    check_offsets = [
        padding  0;
       -padding  0;
        0  padding;
        0 -padding
    ];

    for k = 1:size(check_offsets,1)
        x_check = x + check_offsets(k,1);
        y_check = y + check_offsets(k,2);

        % Convert to map indices with flipped y-axis
        row_check = round((y_bounds(2) - y_check) * 10);
        col_check = round(x_check * 10);

        % Skip out-of-bounds
        if row_check <= 0 || col_check <= 0 || row_check > size(map,1) || col_check > size(map,2)
            continue
        end

        if map(row_check, col_check) == 1
            is_valid = false;
            return
        end
    end

    % --- Near obstacle check (Euclidean distance) ---
    if ~isempty(obstacles)
        d_obs = sqrt((obstacles(:,1) - x).^2 + (obstacles(:,2) - y).^2);
        if any(d_obs <= padding)
            is_valid = false;
            return
        end
    end
end
