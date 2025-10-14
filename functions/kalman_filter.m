function [state_hat, k_gain] = kalman_filter(state_noisy)
    persistent x_hat P A Q R H initialized
    
    if isempty(initialized)
        dt = 0.01;
        A=[1,0,0,dt,0,0;...
        0,1,0,0,dt,0;...
        0,0,1,0,0,dt;...
        0,0,0,1,0,0;...
        0,0,0,0,1,0;...
        0,0,0,0,0,1];
        
        % Process noise covariance Q
        Q = diag([0.1362, 0.08, 0.534, 0.1, 0.1, 0.1]);  % 6x6 diagonal covariance matrix        
        % Measurement noise covariance R
        R = diag([0.1 0.01 0.88]);  
        
        % measurement matrix
        H=[1,0,0,0,0,0;...
        0,1,0,0,0,0;...
        0,0,1,0,0,0];

        x_hat = zeros(6, 1);
        P = 1e6 * eye(6); % large initial uncertainty
        
        
        initialized = true;
    end

    % Prediction
    x_pred = A * x_hat;
    P_pred = A * P * A' + Q;
    
    % Update
    z = state_noisy - H * x_pred;        % innovation
    S = H * P_pred * H' + R;             % innovation covariance
    K = P_pred * H' / S;                 % Kalman gain
    
    x_hat = x_pred + K * z;
    P = (eye(6) - K * H) * P_pred;

    state_hat = x_hat;
    k_gain = K;
end
