% Drone 2D trajectory simulation
% Generates truth data and noisy measurements for EKF validation
% State vector: [x, y, vx, vy]

clear; clc;

%% Simulation parameters
dt = 0.1;           % time step (s)
T = 30;             % total time (s)
t = 0:dt:T;
N = length(t);

%% Process noise (true system disturbances)
sigma_ax = 0.2;     % acceleration noise x (m/s^2)
sigma_ay = 0.2;     % acceleration noise y (m/s^2)

%% Measurement noise (GPS)
sigma_gps_x = 2.0;  % GPS noise x (m)
sigma_gps_y = 2.0;  % GPS noise y (m)

%% Initialize truth state [x, y, vx, vy]
x_true = zeros(4, N);
x_true(:,1) = [0; 0; 5; 2];  % initial position (0,0), velocity (5,2) m/s

%% Simulate true trajectory
for k = 1:N-1
    ax = sigma_ax * randn;
    ay = sigma_ay * randn;
    x_true(1,k+1) = x_true(1,k) + x_true(3,k)*dt;
    x_true(2,k+1) = x_true(2,k) + x_true(4,k)*dt;
    x_true(3,k+1) = x_true(3,k) + ax*dt;
    x_true(4,k+1) = x_true(4,k) + ay*dt;
end

%% Generate noisy GPS measurements
z_meas = zeros(2, N);
for k = 1:N
    z_meas(1,k) = x_true(1,k) + sigma_gps_x * randn;
    z_meas(2,k) = x_true(2,k) + sigma_gps_y * randn;
end

%% Save to CSV
data_truth = [t', x_true'];
data_meas  = [t', z_meas'];

writematrix(data_truth, '../data/truth.csv');
writematrix(data_meas,  '../data/measurements.csv');

disp('Simulation complete. CSVs saved to data folder.');

%% Plot
figure;
plot(x_true(1,:), x_true(2,:), 'b-', 'LineWidth', 2); hold on;
plot(z_meas(1,:), z_meas(2,:), 'r.', 'MarkerSize', 6);
legend('True trajectory', 'Noisy GPS measurements');
xlabel('X (m)'); ylabel('Y (m)');
title('Drone 2D Trajectory');
grid on;