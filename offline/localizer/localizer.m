% localizer code, as seen in java codebase
% Bereket Abraham

% TODO: change dt?, graph map, compare to raw latlon, fix mistakes

addpath('./latlonutm/Codes/matlab');

% constants
dt = (1/10) * 1000; % 10 Hz in ms
wheel_base = 1.13;
utm_zone = 17;
Q = eye(7); % model noise covariance
R = eye(7); % measurement noise covariance

% globals
first_theta = 250;
lat_long = [40.441670, -79.9416362];
[x, y, zone] = ll2utm(lat_long(1), lat_long(2));
first_gps = [x y];
last_gps = [x y];
utm_zone = zone; % fixed
last_encoder = 0;
last_encoder_time = 0;

function llog = log_line(x)
    [lat, lon] = utm2ll(x(1), x(2), utm_zone);
    llog = [lat, lon, x(5)]';
end

function x = scrubAngles(x)
    for i = 5:7
        while (x(i) < -180)
            x(i) = x(i) + 360;
        end
        while (x(i) > 180)
            x(i) = x(i) - 360;
        end
    end
end

function A = model(x)
    % dynamic model
    theta = deg2rad(x(3));
    heading = deg2rad(x(7));
    A = [1, 0, dt*cos(theta), -dt*sin(theta), 0, 0, 0;
         0, 1, dt*sin(theta), dt*cos(theta), 0, 0, 0;
         0, 0, 1, 0, 0, 0, 0;
         0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, dt, 0;
         0, 0, 180/(pi*(wheel_base/sin(heading))), 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 1];
end

function [P_pre, x_pre] = predict_step(P, x)
    A = model(x);
    x_pre = A * x;
    P_pre = eye(7) * P * eye(7)'; % !error! P_pre = A * P * A' + R;
    x_pre = scrubAngles(x_pre);
end

function [P, x] = update_step(C, z, P_pre, x_pre)
    residual = z - (C * x_pre);
    residual = scrubAngles(residual);
    K = P_pre * C' * inv((C * P_pre * C') + Q); % gain
    x = x_pre + (K * residual);
    P = (eye(7) - (K * C)); % !error! P = (eye(7) - (K * C)) * P_pre;
end

function [C, z] = gps_meaurement(lat, long)
    [x, y, ~] = ll2utm(lat, long);
    dx = x - last_gps(1);
    dy = y - last_gps(2);
    last_gps = [x y];
    theta = rad2deg(atan2(dy, dx));
    
    C = zeros(7, 7);
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(5, 5) = 1;
    % ignore small strides
    if (norm([dx dy]) < 0.5)
        C(5, 5) = 0;
    end
    % close the loop
    d = abs(x - first_gps(1)) + abs(y - first_gps(2));
    if (d < 10)
        theta = first_theta;
    end

    z = [x, y, 0, 0, theta, 0 , 0]';
end

% TODO possible precision errors since changes are so small
% maybe batch encoder updates until we get a decent margin
function [C, z] = encoder_meaurement(encoder_dist, encoder_time)
    dx = encoder_dist - last_encoder;
    dt = encoder_time - last_encoder_time;
    C = zeros(7, 7);
    z = zeros(7, 1);
    
    if (dt <= 1)
        return;
    end

    last_encoder = encoder_dist;
    last_encoder_time = encoder_time;
    C(3, 3) = 1;
    z(3) = dx / (dt / 1000);
end

function [C, z] = steering_measurement(heading)
    C = zeros(7, 7);
    C(7, 7) = 1;
    z = zeros(7, 1);
    z(7) = heading;
end

function [C, z] = imu_measurement(R)
    x = R(1, 1);
    x = R(1, 2);
    theta = 90 - rad2deg(atan2(y, x));

    C = zeros(7, 7);
    C(5, 5) = 1;
    z = zeros(7, 1);
    z(5) = theta;
end


% initialize Kalman filter
P = eye(7); % covariance

X = [x;  % X, m, UTM coors
     y;  % Y, m, UTM coors
     0;  % d_Xb, body velocity
     0;  % d_Yb
     first_theta; % theta, deg, global
     0;  % d_theta, deg/s
     0]; % heading, deg, body orientation

trajectory = [X; norm(P)];

load('./roll_logs.mat');

for i = 1:size(logs, 2)
    llog = logs(:, i);
    if llog(1) == 'sensors/gps'
        time = llog(2);
        [C, z] = gps_meaurement(llog(3), llog(4));
    elseif llog(1) == 'sensors/encoder'
        time = llog(2);
        [C, z] = encoder_meaurement(llog(3), time)
    elseif llog(1) == 'sensors/steering'
        time = llog(2);
        [C, z] = steering_measurement(llog(3))
    end

    [P_pre, X_pre] = predict_step(P, X);
    [P, X] = update_step(C, z, P_pre, X_pre)
    trajectory = [trajectory, [X; norm(P)]];
end

% graph results

