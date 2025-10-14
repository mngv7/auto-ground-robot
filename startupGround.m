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

%% Generate and Optimize Waypoints
init_state = [robot.X robot.Y];

x_bound = [1, 52];
y_bound = [1, 41];

no_waypoints = 5;

waypoints = generate_waypoints(logical_map, x_bound, y_bound, no_waypoints, obstacles);

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
if isempty(simout)
    %%Extract pose data
    x_log = out.simout.pose.Data(:,1);
    y_log = out.simout.pose.Data(:,2);
else
    x_log = simout.pose.Data(:,1);
    y_log = simout.pose.Data(:,2);
end

% Compute waypoint capture error
num_waypoints = size(optimal_waypoints, 1);
capture_errors = zeros(num_waypoints, 1);

for i = 1:num_waypoints
    dx = x_log - optimal_waypoints(i,1);
    dy = y_log - optimal_waypoints(i,2);
    dists = sqrt(dx.^2 + dy.^2);
    capture_errors(i) = min(dists); % closest distance to waypoint
end

avg_capture_error = mean(capture_errors);
fprintf('Average Waypoint Capture Error: %.4f m\n', avg_capture_error);

% Compute RMS cross-track error
cte_log = evalin('base','cte_log');
rms_cte = sqrt(mean(cte_log.^2));
fprintf('RMS Cross-Track Error: %.10f m\n', rms_cte);
