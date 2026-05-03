% Load all data
truth = readmatrix('../data/truth.csv');
meas  = readmatrix('../data/measurements.csv');
ekf   = readmatrix('../results/ekf_output.csv');

t       = truth(:,1);
x_true  = truth(:,2:5);
z_meas  = meas(:,2:3);
x_est   = ekf(:,2:5);

% Plot X and Y position
figure;
subplot(2,1,1);
plot(t, x_true(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, z_meas(:,1), 'r.', 'MarkerSize', 4);
plot(t, x_est(:,1),  'g-', 'LineWidth', 2);
legend('Truth','GPS Measurement','C++ EKF Estimate');
xlabel('Time (s)'); ylabel('X position (m)');
title('EKF Validation - X Position');
grid on;

subplot(2,1,2);
plot(t, x_true(:,2), 'b-', 'LineWidth', 2); hold on;
plot(t, z_meas(:,2), 'r.', 'MarkerSize', 4);
plot(t, x_est(:,2),  'g-', 'LineWidth', 2);
legend('Truth','GPS Measurement','C++ EKF Estimate');
xlabel('Time (s)'); ylabel('Y position (m)');
title('EKF Validation - Y Position');
grid on;

% Plot 2D trajectory
figure;
plot(x_true(:,1), x_true(:,2), 'b-', 'LineWidth', 2); hold on;
plot(z_meas(:,1), z_meas(:,2), 'r.', 'MarkerSize', 4);
plot(x_est(:,1),  x_est(:,2),  'g-', 'LineWidth', 2);
legend('Truth','GPS Measurement','C++ EKF Estimate');
xlabel('X (m)'); ylabel('Y (m)');
title('2D Drone Trajectory - EKF Validation');
grid on;