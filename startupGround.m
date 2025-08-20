%% close previously open model
close_system('sl_groundvehicleDynamics',0);
 

%% add toolboxes to path
homedir = pwd; 
addpath( genpath(strcat(homedir,[filesep,'toolboxes'])));
addpath('functions');

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

cd(homedir);

%% open current model
open_system('sl_groundvehicleDynamics'); %differential robot

cd(homedir);

%% Generate and Optimize Waypoints

init_state = [robot.X robot.Y];

X = randi([-500, 500], 10, 1);
Y = randi([-500, 500], 10, 1);
waypoints = [X Y];

% optimal_waypoints = optimize_waypoints(waypoints, init_state);

[optimal_waypoints_order, min_cost] = optimize_waypoints(waypoints);

optimal_waypoints = [];

for i = 1:size(waypoints, 1)
    optimal_waypoints(i, :) = waypoints(optimal_waypoints_order(i), :);
end