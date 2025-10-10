function [v_control, w_control, stop] = rvwp(waypoints, state, ranges, detections)
    % RVWP Controller with persistent waypoint index and cross-track error logging
    % waypoints: Nx2 array [x, y]
    % state: [x, y, psi]
    % stop: 1 if last waypoint reached

    % ===== Parameters =====
    k = 8.0;                  % heading error gain
    base_look_ahead = 3.0;    % fixed look-ahead distance
    v_max = 1.0;             % max linear velocity
    capture_threshold = 0.2; % waypoint capture threshold

    x = state(1);
    y = state(2);
    psi = state(3);

    Wx = waypoints(:,1)'; 
    Wy = waypoints(:,2)';
    N = length(Wx);

    % Persistent variables
    persistent n cte_log
    if isempty(n)
        n = 1;
    end
    if isempty(cte_log)
        cte_log = [];
    end

    % Check if last waypoint reached
    if n >= N
        v_control = 0;
        w_control = 0;
        stop = 1;
        return
    else
        stop = 0;
    end

    % Distance to next waypoint
    d_next = sqrt((x - Wx(n+1))^2 + (y - Wy(n+1))^2);
    if d_next < capture_threshold
        n = n + 1;
        if n >= N
            v_control = 0;
            w_control = 0;
            stop = 1;
            return
        end
    end

    % Core RVWP logic
    Ru = sqrt((x - Wx(n))^2 + (y - Wy(n))^2);
    theta = atan2(Wy(n+1) - Wy(n), Wx(n+1) - Wx(n));
    theta_u = atan2(y - Wy(n), x - Wx(n));
    beta = atan2(sin(theta_u - theta), cos(theta_u - theta));

    R = sqrt(max(Ru^2 - (Ru*sin(beta))^2, 1e-6)); 
    S_x = Wx(n) + (base_look_ahead + R)*cos(theta);
    S_y = Wy(n) + (base_look_ahead + R)*sin(theta);

    psi_d = atan2(S_y - y, S_x - x);
    error = atan2(sin(psi_d - psi), cos(psi_d - psi));

    % Controls
    w_control = k * error;

    % Adaptive linear velocity based on heading alignment and distance
    alignment_factor = cos(error); % ranges from -1 to 1
    distance_factor = min(d_next / base_look_ahead, 1); % scale down near waypoint
    v_control = v_max * alignment_factor * distance_factor;
    v_control = max(min(v_control, v_max), 0.5); % clamp to safe range

    % Cross-track error logging
    cross_track_error = Ru * sin(beta);
    cte_log(end+1) = cross_track_error;

    assignin('base', 'cte_log', cte_log);
end
