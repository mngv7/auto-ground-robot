function [v, w, stop, cross_track_error] = vfh_pid(waypoints, state, lidar_detections, lidar_scan_angle)
    coder.extrinsic('angdiff');

    stop = 0;
    v = 0.4;
    error = 0.0;
    cross_track_error = 0;

    persistent WP_index vfh prev_error integ_error prev_time
    if isempty(WP_index)
        WP_index = 1;

        % Initialize VFH Controller
        vfh = controllerVFH('UseLidarScan', false);
        vfh.DistanceLimits   = [ 0.05, 5 ];
        vfh.RobotRadius      = 0.25;  % Inflate robot size
        vfh.SafetyDistance   = 0.1;   % Keep extra clearance from walls
        vfh.MinTurningRadius = 0.01;   % Smoother turns
        % vfh.HistogramThresholds = [ 1, 2 ];
        prev_error = 0.0;
        integ_error = 0.0;
        prev_time = 0.0;
    end

    % PID gains
    Kp = 0.7;
    Ki = 0.05;
    Kd = 0.1;
    w = 0.0;
    dt = 0.05;  % Time step
    Xd = waypoints(:,1);
    Yd = waypoints(:,2);

    % Extract vehicle state
    x = state(1);
    y = state(2); 
    psi = state(3);

    % Desired heading to current waypoint
    psi_star = atan2((Yd(WP_index) - y), (Xd(WP_index) - x));
    % Distance to current waypoint

    % Compute Cross-Track Error
    if WP_index == 1
        x_prev = Xd(1); y_prev = Yd(1);
    else
        x_prev = Xd(WP_index-1); y_prev = Yd(WP_index-1);
    end
    x_curr = Xd(WP_index); y_curr = Yd(WP_index);

    cross_track_error = abs((y_curr-y_prev)*x - (x_curr-x_prev)*y + x_curr*y_prev - y_curr*x_prev) / ...
                        sqrt((y_curr-y_prev)^2 + (x_curr-x_prev)^2);
    
    % ----- Wall Avoidance Enhancement -----
    if ~isempty(lidar_detections)
        % Artificially inflate obstacle proximity (makes robot steer away earlier)

        % Get avoidance direction from VFH
        targetDir = psi_star;  % toward current waypoint
        steerDir = vfh(lidar_detections, lidar_scan_angle, targetDir);

        if ~isnan(steerDir)
            psi_star = steerDir;  % override heading if obstacle nearby
        else
            v = 0.1;
            w = 2;
            return
        end
    end
    % -------------------------------------

    % Compute distance to current waypoint
    distance_to_current_waypoint = sqrt((x - Xd(WP_index))^2 + (y - Yd(WP_index))^2);


    % Stop condition
    if WP_index == length(Yd) && distance_to_current_waypoint < 0.9
        WP_index = 1;
        stop = 1;
    end

    % Move to next waypoint
    if distance_to_current_waypoint < 0.5 && WP_index < length(Xd)
        WP_index = WP_index + 1;
        % clear pid gains
        integ_error = 0.0;  % Reset integral error for new waypoint
        prev_error = 0.0;
        integ_error = 0.0;
        prev_time = 0.0;
    end

    % PID heading control
    error = double(angdiff(psi, psi_star));

    if isfinite(psi_star)
        P = Kp * error;
        integ_error = integ_error + error * dt;
        I = Ki * integ_error;
        D = Kd * (error - prev_error) / dt;
        prev_error = error;
        w = P + I + D;
    else
        v = 0.1;
        w = 0.5;
    end
end
