function [x_est, P_out] = ekf_drone(x_init, P_init, z_meas, dt, Q, R)
% Extended Kalman Filter for 2D drone position/velocity estimation
% State: [x, y, vx, vy]
% Measurement: [x, y] from GPS
%
% Inputs:
%   x_init  - initial state [4x1]
%   P_init  - initial covariance [4x4]
%   z_meas  - measurements [Nx2]
%   dt      - time step (scalar)
%   Q       - process noise covariance [4x4]
%   R       - measurement noise covariance [2x2]
%
% Outputs:
%   x_est   - estimated states [Nx4]
%   P_out   - final covariance [4x4]

N = size(z_meas, 1);

% State transition matrix
F = [1, 0, dt, 0;
     0, 1, 0,  dt;
     0, 0, 1,  0;
     0, 0, 0,  1];

% Measurement matrix
H = [1, 0, 0, 0;
     0, 1, 0, 0];

% Initialize
x = x_init;
P = P_init;
x_est = zeros(N, 4);

for k = 1:N
    % Predict
    x = F * x;
    P = F * P * F' + Q;

    % Update
    z = z_meas(k, :)';
    y = z - H * x;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(4) - K * H) * P;

    % Store
    x_est(k, :) = x';
end

P_out = P;
end