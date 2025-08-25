function [v_control, w_control, stop] = rvwp(waypoints, state)
    %#codegen
    % RVWP Controller with persistent waypoint index
    % waypoints: Nx2 array [x, y]
    % state: [x, y, psi]
    % stop: 1 if last waypoint reached

    % ===== Parameters =====
    k = 2.0;                 
    d_look_ahead = 2.0;      
    v_max = 10.0;             
    w_max = 5.0;             
    distance_threshold = 0.5; 

    x = state(1);
    y = state(2);
    psi = state(3);

    Wx = waypoints(:,1)'; 
    Wy = waypoints(:,2)';
    N = length(Wx);

    % ===== Persistent waypoint index =====
    persistent n
    if isempty(n)
        n = 1;
    end

    % ===== Check if last waypoint reached =====
    if n >= N
        v_control = 0;
        w_control = 0;
        stop = 1; % Trigger Stop Simulation block
        return
    else
        stop = 0;
    end

    % ===== Distance to next waypoint =====
    d_next = sqrt((x - Wx(n+1))^2 + (y - Wy(n+1))^2);
    if d_next < distance_threshold
        n = n + 1;
        if n >= N
            v_control = 0;
            w_control = 0;
            stop = 1;
            return
        end
    end

    % ===== Core RVWP logic =====
    Ru = sqrt((x - Wx(n))^2 + (y - Wy(n))^2);
    theta = atan2(Wy(n+1) - Wy(n), Wx(n+1) - Wx(n));
    theta_u = atan2(y - Wy(n), x - Wx(n));
    beta = atan2(sin(theta_u - theta), cos(theta_u - theta));

    R = sqrt(max(Ru^2 - (Ru*sin(beta))^2, 1e-6)); 
    S_x = Wx(n) + (R + d_look_ahead)*cos(theta);
    S_y = Wy(n) + (R + d_look_ahead)*sin(theta);

    psi_d = atan2(S_y - y, S_x - x);
    error = atan2(sin(psi_d - psi), cos(psi_d - psi));

    % ===== Controls =====
    w_control = max(min(k * error, w_max), -w_max);
    v_control = v_max; %min(max(v_control, 0.5), v_max);
end
