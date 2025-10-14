%% Close previously open model
close_system('sl_groundvehicleDynamics', 0);
clear all; clc;
%% Add toolboxes to path
homedir = pwd; 
addpath(genpath(fullfile(homedir, 'toolboxes')));
addpath('functions');

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;
cd(homedir);

%% Open current model
open_system('sl_groundvehicleDynamics'); % dif ferential robot
load('obstacles_air_ground.mat')
load('complexMap_air_ground.mat')

%% LiDAR configuration
scan_angles = linspace(-pi/4,pi/4,50);

%% Generate and Optimize Waypoints
init_state = [robot.X robot.Y];


x_bound = [0, 52];
y_bound = [0, 41];

wall_padding = [2, -2];

no_waypoints = 5;

X = randi(x_bound + wall_padding, no_waypoints , 1);
Y = randi(y_bound + wall_padding, no_waypoints , 1);

waypoints = [X Y];

optimal_waypoints = optimize_waypoints(waypoints, init_state);
optimal_waypoints = [init_state; optimal_waypoints];

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
obstacles = [];