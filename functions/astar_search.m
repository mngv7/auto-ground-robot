function [distance, path] = astar_search(start, goal, map, wp_interval)

%% Convert start and goal to map indices
[start_x, start_y] = deal(start(1), start(2));
[goal_x, goal_y] = deal(goal(1), goal(2));

[start_map_x, start_map_y] = convert_to_map(start_x, start_y);
[goal_map_x, goal_map_y] = convert_to_map(goal_x, goal_y);

start_idx = [start_map_x, start_map_y];
goal_idx  = [goal_map_x, goal_map_y];

% Initialize A* structures
open_set = start_idx;                  % nodes to explore
came_from = containers.Map();          % to reconstruct path

g_score = inf(size(map));              % g: cost from start
f_score = inf(size(map));              % f: g + heuristic
g_score(start_map_y, start_map_x) = 0; % note: row = y, col = x (fuckass logical map)
f_score(start_map_y, start_map_x) = heuristic(start_idx, goal_idx);

% Allowed moves
moves = [ -1,  0;
           1,  0;
           0, -1;
           0,  1;
          -1, -1;
          -1,  1;
           1, -1;
           1,  1];

% --- A* search loop ---
while ~isempty(open_set)
    % Pick node in open set with smallest f_score
    min_f = inf;
    current_idx = [];
    for i = 1:size(open_set,1)
        node = open_set(i,:);
        fs = f_score(node(2), node(1));
        if fs < min_f
            min_f = fs;
            current_idx = node;
            idx_in_open = i;
        end
    end
    
    if isequal(current_idx, goal_idx)
        % Goal reached: reconstruct path
        path_map = reconstruct_path(came_from, current_idx);
        % Convert back to original coordinates
        path = map_to_original(path_map);
        distance = sum(sqrt(sum(diff(path).^2,2)));
        
        % Reduce to wp_interval points
        if length(path) > wp_interval
            idxs = round(linspace(1, length(path), wp_interval));
            path = path(idxs,:);
        end
        return
    end
    
    % Remove current from open set
    open_set(idx_in_open,:) = [];
    
    % Explore neighbors
    for m = 1:size(moves,1)
        neighbor = current_idx + moves(m,:);
        nx = neighbor(1); ny = neighbor(2);
        % Check bounds
        if nx < 1 || nx > size(map,2) || ny < 1 || ny > size(map,1)
            continue
        end
        % Check wall
        if map(ny, nx) == 1
            continue
        end
        % Check near wall
        near = 0;
        for l = 1:size(moves,1)
            padded_neigbour = neighbor + (moves(l,:) * 10);
            padded_nx = padded_neigbour(1); padded_ny = padded_neigbour(2);
            if map(padded_ny, padded_nx) == 1
                near = 1;
            end
        end
        if near == 1
            continue
        end
        tentative_g = g_score(current_idx(2), current_idx(1)) + norm(neighbor - current_idx);
        if tentative_g < g_score(ny, nx)
            % Update path
            key = sprintf('%d_%d', nx, ny);
            came_from(key) = current_idx;
            g_score(ny, nx) = tentative_g;
            f_score(ny, nx) = tentative_g + heuristic(neighbor, goal_idx);
            % Add to open set if not already
            if ~ismember(neighbor, open_set, 'rows')
                open_set = [open_set; neighbor];
            end
        end
    end
end

% no path found
distance = inf;
path = [];

end

%% Helper functions

function [new_x, new_y] = convert_to_map(x, y)
    new_x = round(x * 10);
    new_y = round(410 - y * 10);
end

function h = heuristic(node, goal)
    h = norm(node - goal);
end

function path = reconstruct_path(came_from, current)
    path = current;
    key = sprintf('%d_%d', current(1), current(2));
    while isKey(came_from, key)
        prev = came_from(key);
        path = [prev; path];
        key = sprintf('%d_%d', prev(1), prev(2));
    end
end

function coords = map_to_original(path_map)
    coords = zeros(size(path_map));
    for i = 1:size(path_map,1)
        coords(i,1) = path_map(i,1)/10; % x
        coords(i,2) = (410 - path_map(i,2))/10; % y
    end
end
