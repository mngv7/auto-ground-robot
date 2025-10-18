%% Close previously open model
close_system('sl_groundvehicleDynamics', 0);
%% Add toolboxes to path
homedir = pwd; 
addpath(genpath(fullfile(homedir, 'toolboxes')));
addpath('functions');

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;
cd(homedir);

%% Open current model
open_system('sl_groundvehicleDynamics'); % differential robot
load('obstacles_air_ground.mat')
load('complexMap_air_ground.mat')
%% Mask occupancy with obstacles for lidar scanner
lidar_map = copy(map);
w = 0.4;
h = 0.4;
for i=1:size(obstacles, 1)
    xi = obstacles(i,1);
    yi = obstacles(i,2);
    % Get grid indices from world positions
    idxMin = world2grid(lidar_map, [xi - w, yi - h]);
    idxMax = world2grid(lidar_map, [xi + w, yi + h]);
    rowRange = min(idxMin(1), idxMax(1)) : max(idxMin(1), idxMax(1));
    colRange = min(idxMin(2), idxMax(2)) : max(idxMin(2), idxMax(2));
    
    [cc, rr] = meshgrid(colRange, rowRange);
    gridIdx = [rr(:), cc(:)];
    
    % Set grid cells as occupied
    setOccupancy(lidar_map, gridIdx, true, 'grid');

end
inflate(lidar_map, 0.2);
show(lidar_map)
%% Sensor configuration
scan_angles = linspace(-pi/3,pi/3,50);
%% Generate and Optimize Waypoints
init_state = [robot.X robot.Y];


x_bound = [0, 52];
y_bound = [0, 41];

wall_padding = [2, -2];

no_waypoints = 5;

waypoints = generate_waypoints(logical_map, x_bound, y_bound, no_waypoints, []);
%%
[optimal_waypoints, paths] = optimize_waypoints(waypoints, init_state, logical_map);
optimal_waypoints = [init_state; optimal_waypoints];

% Make paths symmetric
for i = 1:size(paths,1)
    for j = 1:size(paths,2)
        if all(paths(i,j,:,:) == 0, 'all') && any(paths(j,i,:,:) ~= 0, 'all')
            paths(i,j,:,:) = flip(paths(j,i,:,:), 3);
        end
    end
end

waypoints = [init_state; waypoints];

complete_path = [];

% Construct the complete path using the optimally ordered waypoints
% and 4D paths matrix
for i = 1:size(optimal_waypoints, 1)-1
    curr = optimal_waypoints(i,:); % Current waypoint
    next = optimal_waypoints(i+1,:); % Next waypoint
    
    idx_curr = find(ismembertol(waypoints, curr, 'ByRows', true, 'DataScale', 1e-6), 1);
    idx_next = find(ismembertol(waypoints, next, 'ByRows', true, 'DataScale', 1e-6), 1);
    
    if isempty(idx_curr) || isempty(idx_next)
        warning('Waypoint not found: curr = [%f, %f], next = [%f, %f]', ...
                curr(1), curr(2), next(1), next(2));
        continue;
    end
    
    path_segment = squeeze(paths(idx_curr, idx_next, :, :));
    
    valid_rows = any(path_segment ~= 0, 2);
    path_segment = path_segment(valid_rows, :);
    
    if isempty(path_segment)
        warning('Empty path between waypoints %d and %d', idx_curr, idx_next);
        continue;
    end
    
    if i < size(optimal_waypoints, 1)-1
        path_segment = path_segment(1:end-1, :);
    end
    
    complete_path = [complete_path; path_segment];
end

complete_path = complete_path(any(complete_path ~= 0, 2), :);

if ~isempty(complete_path)
    complete_path(1,:) = optimal_waypoints(1,:);
    complete_path(end,:) = optimal_waypoints(end,:);
end

%% Sanity check

imagesc(logical_map);       % visualize the map
colormap(flipud(gray));  % flip the colormap upside down
axis equal tight;
xlabel('X'); ylabel('Y');
title('Logical Map');

%% Run simulation and measure time
tic;
out = sim('sl_groundvehicleDynamics');
mission_time = toc;

fprintf('Trip time: %.4f s\n', mission_time)

x_log = clean_pose.Data(1,:);
y_log = clean_pose.Data(2,:);


%% Compute waypoint capture error
key_waypoints = optimal_waypoints(2:end, :);  % skip initial state
all_waypoints = complete_path(2: end, :);

num_key_waypoints = size(key_waypoints, 1);
capture_errors_key = zeros(num_key_waypoints, 1);

num_wayponts = size(all_waypoints, 1);
capture_errors = zeros(num_wayponts, 1);

% Error for key waypoints
for i = 1:num_key_waypoints
    dx_k = x_log - key_waypoints(i,1);
    dy_k = y_log - key_waypoints(i,2);
    dists = sqrt(dx_k.^2 + dy_k.^2);
    capture_errors_key(i) = min(dists); % closest distance to this key waypoint
end

% Error for complete path
for i = 1:size(complete_path, 1)
    dx = x_log - complete_path(i,1);
    dy = y_log - complete_path(i,2);
    dists = sqrt(dx.^2 + dy.^2);
    capture_errors(i) = min(dists); % closest distance to this path waypoint
end

avg_capture_error_key = mean(capture_errors_key);
fprintf('Average Key Waypoint Capture Error: %.4f m\n', avg_capture_error_key);

avg_capture_error_complete = mean(capture_errors);
fprintf('Average Complete Path Capture Error: %.4f m\n', avg_capture_error_complete);


%% Compute RMS cross-track error
cte_time = cross_track_error.Time;
cte_data = cross_track_error.Data;

% Pop first and last value (will be NaN).
cte_data(1) = [];
cte_time(1) = [];

cte_data(end) = [];
cte_time(end) = [];

rms_cte = sqrt(mean(cte_data.^2));
fprintf('RMS Cross-Track Error: %.6f m\n', rms_cte);

% Plot
figure;
hold on
plot(cte_time, cte_data, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Cross-Track Error [m]');
title('Robot Cross-Track Error Over Time');
grid on;

% Plot RMS as a horizontal line
plot(cte_time, rms_cte*ones(size(cte_time)), 'r--', 'LineWidth', 1.5);

legend('Cross-Track Error', 'RMS Error');
hold off

%% Compute closest point on trajectory to each obstacle
num_obstacles = size(obstacles, 1);
closest_points = zeros(num_obstacles, 2);
min_dists = zeros(num_obstacles, 1);

for i = 1:num_obstacles
    obs_x = obstacles(i, 1);
    obs_y = obstacles(i, 2);

    % Compute distances from robot trajectory to this obstacle
    dx = x_log - obs_x;
    dy = y_log - obs_y;
    dists = sqrt(dx.^2 + dy.^2);

    % Find the minimum distance and the corresponding point
    [min_dists(i), idx] = min(dists);
    closest_points(i, :) = [x_log(idx), y_log(idx)];
end

% Display summary
fprintf('\nClosest distances to each obstacle:\n');
for i = 1:num_obstacles
    fprintf('Obstacle %d: min distance = %.4f m at point (%.3f, %.3f)\n', ...
        i, min_dists(i), closest_points(i,1), closest_points(i,2));
end

%% Plot obstacle distances
figure;
show(map);  % map in world coordinates
hold on;

plot(x_log, y_log, 'b', 'LineWidth', 1.5);

% Plot obstacles as colored squares and number them
for i = 1:size(obstacles, 1)
    % Determine color
    switch obstacles(i,3)
        case 1
            c = [1 0 0]; % red
        case 2
            c = [0 1 0]; % green
        case 3
            c = [0 0 1]; % blue
        otherwise
            c = [0 0 0]; % fallback black
    end
    
    % Plot square
    plot(obstacles(i,1), obstacles(i,2), 's', ...
        'MarkerSize', 10, 'MarkerFaceColor', c, 'MarkerEdgeColor','k','LineWidth',1.2);
    
    % Number the obstacle
    text(obstacles(i,1)+0.3, obstacles(i,2)+0.3, sprintf('%d', i), ...
         'Color','k','FontWeight','bold','FontSize',10);
end

% Plot closest points as black circles
scatter(closest_points(:,1), closest_points(:,2), 100, 'ko', 'filled');

% Number the closest point
for i = 1:size(closest_points, 1)
    text(closest_points(i,1)+0.3, closest_points(i,2)+0.3, sprintf('%d', i), ...
         'Color','k','FontWeight','bold','FontSize',10);
end

% Draw lines connecting each obstacle to its closest point
for i = 1:size(obstacles, 1)
    plot([obstacles(i,1), closest_points(i,1)], ...
         [obstacles(i,2), closest_points(i,2)], ...
         'r', 'LineWidth', 1);  % dashed black line
end

% Legend
legendEntries = {'Closest Points', 'Obstacles'};
legend(legendEntries);

xlabel('X [m]'); ylabel('Y [m]');
title('Robot Trajectory and Obstacles with Closest Points');
axis equal;
grid on;

%% Increase figure font sizes
% Desired font sizes
titleFontSize = 16;
labelFontSize = 14;
tickFontSize  = 12;
legendFontSize = 12;

% Get handles to all open figures
figHandles = findall(0, 'Type', 'figure');

for f = 1:length(figHandles)
    figure(figHandles(f)); % make figure current

    % Update axes
    ax = findall(figHandles(f), 'Type', 'axes');
    for k = 1:length(ax)
        % Increase title and labels font size
        t = get(ax(k), 'Title');
        t.FontSize = titleFontSize;
        t.FontWeight = 'bold';

        xl = get(ax(k), 'XLabel');
        xl.FontSize = labelFontSize;
        xl.FontWeight = 'bold';

        yl = get(ax(k), 'YLabel');
        yl.FontSize = labelFontSize;
        yl.FontWeight = 'bold';

        % Increase tick label font size
        ax(k).FontSize = tickFontSize;

        % Update legends if present
        lg = findall(ax(k), 'Type', 'Legend');
        for j = 1:length(lg)
            lg(j).FontSize = legendFontSize;
        end
    end
end
