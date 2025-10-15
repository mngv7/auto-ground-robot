function [v_control, w_control, stop] = rvwp(waypoints, state, detections)
% RVWP Controller using external waypoint_selector
% waypoints: Nx2 [x y]
% state    : [x y psi]

    % ===== Parameters =====
    k = 8.0;                  % heading error gain
    base_look_ahead = 3.0;    % fixed look-ahead distance
    v_max = 2;                % max linear velocity
    capture_threshold = 0.7;  % waypoint capture threshold

    x   = state(1);
    y   = state(2);
    psi = state(3);

    Wx = waypoints(:,1)';
    Wy = waypoints(:,2)';
    N  = numel(Wx);

    % ===== Persistent for logging =====
    persistent cte_log
    if isempty(cte_log), cte_log = []; end

    % ===== Waypoint selection =====
    [n, stop] = planner(waypoints, [x y], capture_threshold, detections, false);
    if stop || n >= N
        v_control = 0;
        w_control = 0;
        stop = 1;
        return
    end

    % ===== Distance to next waypoint =====
    d_next = hypot(x - Wx(n+1), y - Wy(n+1));

    % ===== Core RVWP logic =====
    Ru      = hypot(x - Wx(n), y - Wy(n));
    theta   = atan2(Wy(n+1) - Wy(n), Wx(n+1) - Wx(n));
    theta_u = atan2(y - Wy(n),       x - Wx(n));
    beta    = atan2(sin(theta_u - theta), cos(theta_u - theta));  % wrap

    % geometric look-ahead point along the path
    R   = sqrt(max(Ru^2 - (Ru*sin(beta))^2, 1e-6));
    S_x = Wx(n) + (base_look_ahead + R)*cos(theta);
    S_y = Wy(n) + (base_look_ahead + R)*sin(theta);

    % heading error
    psi_d = atan2(S_y - y, S_x - x);
    err   = atan2(sin(psi_d - psi), cos(psi_d - psi));

    % ===== Controls =====
    w_control = k * err;

    % adaptive linear velocity (slow when misaligned or near the waypoint)
    alignment_factor = max(cos(err), 0);                 % in [0,1]
    distance_factor  = min(d_next / base_look_ahead, 1); % in [0,1]
    v_control = v_max * alignment_factor * distance_factor;
    v_control = max(min(v_control, v_max), 0.5);         % clamp to safe range

    % ===== Cross-track error logging =====
    cross_track_error = Ru * sin(beta);
    cte_log(end+1) = cross_track_error;
    assignin('base', 'cte_log', cte_log);
end
