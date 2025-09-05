% Close previously open model
close_system('sl_groundvehicleDynamics', 0);

% Add toolboxes and functions to path
homedir = pwd; 
addpath(genpath(fullfile(homedir,'toolboxes')));
addpath('functions');

% Start Mobile Robotics Toolbox
cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;
cd(homedir);

% Open model
open_system('sl_groundvehicleDynamics'); 

% Simulation parameters
numTests = 100;         % number of test runs
stopTime = 1000;         % simulation stop time [s]

% Preallocate arrays for metrics
avg_errors = zeros(numTests,1);
rms_errors = zeros(numTests,1);

% Loop over multiple tests
for k = 1:numTests
    fprintf('Running simulation %d/%d...\n', k, numTests);

    % Generate random initial state and waypoints
    init_state = [robot.X robot.Y];  % initial robot position
    X = randi([-500, 500], 10, 1);
    Y = randi([-500, 500], 10, 1);
    waypoints = [X Y];

    % Optimize waypoints
    optimal_waypoints = optimize_waypoints(waypoints, init_state);

    % Assign optimized waypoints to model workspace
    assignin('base', 'waypoints', optimal_waypoints);

    % Run Simulink simulation
    simOut = sim('sl_groundvehicleDynamics', 'StopTime', num2str(stopTime));

    % Extract pose data
    x_log = simOut.get('simout').pose.Data(:,1);
    y_log = simOut.get('simout').pose.Data(:,2);

    % Compute waypoint capture error
    num_waypoints = size(optimal_waypoints, 1);
    capture_errors = zeros(num_waypoints, 1);

    for i = 1:num_waypoints
        dx = x_log - optimal_waypoints(i,1);
        dy = y_log - optimal_waypoints(i,2);
        dists = sqrt(dx.^2 + dy.^2);
        capture_errors(i) = min(dists); % closest distance to waypoint
    end

    avg_errors(k) = mean(capture_errors);

    % Compute RMS cross-track error
    cte_log = evalin('base','cte_log');  % assuming cte_log is in base workspace
    rms_errors(k) = sqrt(mean(cte_log.^2));
end

% Compute mean metrics across all simulations
mean_avg_error = mean(avg_errors);
mean_rms_error = mean(rms_errors);

fprintf('\nSimulation Summary:\n');
fprintf('Mean Average Waypoint Capture Error: %.2f m\n', mean_avg_error);
fprintf('Mean RMS Cross-Track Error: %.2f m\n', mean_rms_error);

% Optionally save metrics
save('simulation_metrics.mat','avg_errors','rms_errors','mean_avg_error','mean_rms_error');
