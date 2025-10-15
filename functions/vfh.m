function [v, w] = vfh(ranges, scan_angles, state, obstacle_threshold, obstacle_detection)
%#codegen
% Reactive circular obstacle avoidance using Lidar data.
%
% Inputs:
%   ranges              1xN lidar range readings [m]
%   scan_angles         1xN corresponding beam angles [rad]
%   state               [x, y, theta] robot pose
%   obstacle_threshold  distance threshold for obstacle detection [m]
%   obstacle_detection  boolean flag (1 if any obstacle detected)
%
% Outputs:
%   v  linear velocity [m/s]
%   w  angular velocity [rad/s]

% Parameters (tunable)
v_max   = 0.4;   % maximum forward speed [m/s]
w_max   = 1.2;   % maximum angular speed [rad/s]
safety_margin = 0.1; % inflation around obstacle [m]

% Default motion: move forward
v = v_max;
w = 0;

% Only act if obstacle is detected
if obstacle_detection

    % Find the closest obstacle within threshold
    [min_range, idx] = min(ranges);

    if min_range < obstacle_threshold
        % Angle of the nearest obstacle
        angle_obs = scan_angles(idx);

        % Compute tangential direction to go around the circle
        % If obstacle is to the left (angle > 0) → steer right, and vice versa
        if angle_obs >= 0
            avoid_angle = angle_obs - pi/2;  % turn right around obstacle
        else
            avoid_angle = angle_obs + pi/2;  % turn left around obstacle
        end

        % Compute desired heading (in robot frame)
        desired_heading = wrapToPi(avoid_angle);

        % Convert heading error into angular velocity
        w = max(-w_max, min(w_max, 2.0 * desired_heading));

        % Slow down when close to obstacle or turning sharply
        v = v_max * (min_range / obstacle_threshold) * exp(-abs(desired_heading));

        % Ensure nonnegative speed
        v = max(0.05, v);
    end
end

end
