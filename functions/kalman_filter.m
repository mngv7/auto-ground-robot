function [state_hat] = kalman_filter(state_noisy)
    % Multi-dimensional Kalman filter
    % state_noisy : measurement vector (m×1)
    % state_hat   : filtered state estimate (6×1)

    persistent state_prev P A H Q R initialized

    if (isempty(initialized))
        dt = 0.05;
        A = [1, 0, 0, dt, 0,  0;
             0, 1, 0, 0,  dt, 0;
             0, 0, 1, 0,  0,  dt;
             0, 0, 0, 1,  0,  0;
             0, 0, 0, 0,  1,  0;
             0, 0, 0, 0,  0,  1];
        sigma = 1e-1;
        sigma_theta = 2;
        sigmadot = 5e-2;
        Q = diag([sigma, sigma, sigma_theta, ...
            sigmadot, sigmadot, sigmadot]);
        R = 0.5 * eye(3);
        H = [1, 0, 0, 0, 0, 0;
             0, 1, 0, 0, 0, 0;
             0, 0, 1, 0, 0, 0];
        
        % Initially estimate state to zero
        state_prev = [0; 0; 0; 0; 0; 0];
        % Initially large measurement uncertainty
        P = eye(6) * 1000;
        initialized = true;
    end


    % Kalman Filter Implementation
    % prediction based on current state, model, no input
    x_predict = A * state_prev;
    P_predict = A * P * A' + Q;

    % update
    % S = residual covariance (combined uncertainty in
    % measurement and prediction)
    % K = kalman gain (balanced between prediction and
    % measurement recursively
    % P = covariance updated to increase certainty after
    % measurement
    % y = measurement residuals
    S = H * P_predict * H' + R;
    K = (P * H') * inv(S);
    z = state_noisy - H * x_predict;
    P = (eye(6) - K * H) * P_predict;
    
    state_prev = x_predict + K * z;
    
    state_hat = state_prev;

end