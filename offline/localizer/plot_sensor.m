clear;
close all;

load('./roll_logs_combined.mat');
start_time = double(start_time);
% ['imu_linear_no_grav', = 0 linear accel
% 'imu_ang_pos', = 1 angle position
% 'imu_ang_vel', = 2 angular velocity
% 'gps', = 3 
% 'encoder', = 4 encoder distance in m
% 'steering', = 5 steering wheel angle
% 'imu_temp'] = 6 compass angle, bugs

imu_linear_acc = logs(logs(:,1) == 0, :);
time = (imu_linear_acc(:, 2) - start_time) / 1000.0;

% figure();
% plot(time, imu_linear_acc(:, 3), 'r');
% figure();
% plot(time, imu_linear_acc(:, 4), 'b');

imu_ang_pos = logs(logs(:,1) == 1, :);
time2 = (imu_ang_pos(:, 2) - start_time) / 1000.0;

r21 = imu_ang_pos(:, 6);
r11 = imu_ang_pos(:, 3);
yaw = atan2(r21, r11);

figure();
plot(time2, yaw, 'r');


% load('localizer_v3.mat', 'trajectory');
