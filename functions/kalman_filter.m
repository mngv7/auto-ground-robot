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
        Q = diag([0.1362, 0.1362, 0.72, 0.52, 0.52, 1]);  % 6x6 diagonal covariance matrix        
        % Measurement noise covariance R
        R = diag([1, 1, 0.5])*1e2; 
        % Process noise covariance Q
        % Q = diag([0.0862, 0.075, 0.7826, 0.37, 0.37, 0.42]);  % 6x6 diagonal covariance matrix        
        % Measurement noise covariance R
        % R = diag([1.05e3 1.05e3 0.88e3]);  
        % sigma = 0.013;
        % sigma_dot = sigma;
        % Q = diag([sigma, sigma, sigma, sigma_dot, sigma_dot, sigma_dot]);
        % Const_R = 100;
        % R = Const_R .* eye(3);
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