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

for i = 1:5
    curr = optimal_waypoints(i,:); % 2 2
    next = optimal_waypoints(i+1,:); % 2 15
    idx_curr = find(ismember(waypoints, curr, 'rows'));
    idx_next = find(ismember(waypoints, next, 'rows'));
    path = squeeze(paths(idx_curr, idx_next, 1:10, :));
    if i ~= 5
        path(end,:) = [];
    end
    complete_path = [complete_path; path];
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
