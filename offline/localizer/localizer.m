function [trajectory] = localizer()
    % localizer code, as seen in java codebase
    % Bereket Abraham

    % TODO: compare to raw latlon, granular position truth?
    % TODO: UNITS, use IMU body accel

    addpath('./latlonutm/Codes/matlab');

    global wheel_base
    global last_gps
    global last_encoder
    global last_encoder_time
    global last_time
    save_data = true;

    % constants
    ref_dt = (1/10) * 1000; % 10 Hz in ms
    % dt of the controller, not the filter
    wheel_base = 1.13;
    utm_zone = 17;
    first_theta = 250;
    lat_long = [40.441670, -79.9416362];

    [x, y, zone] = ll2utm(lat_long(1), lat_long(2));
    first_gps = [x y];
    last_gps = [x y];
    utm_zone = zone; % fixed
    last_encoder = -1;

    % initialize Kalman filter
    P = eye(7); % covariance

    X = [x;  % X, m, UTM coors
         y;  % Y, m, UTM coors
         0;  % d_Xb, body velocity
         0;  % d_Yb
         first_theta; % theta, deg, global
         0;  % d_theta, deg/s
         0]; % heading, deg, body orientation

    trajectory = [X; lat_long'; norm(P)];

    load('./roll_logs_combined.mat');
    start_time = double(start_time);
    last_encoder_time = start_time;
    last_time = start_time;

    for i = 1:size(logs, 1)
        llog = logs(i, :);
        name = map_names{llog(1) + 1};
        time = llog(2);

        if strcmp(name, 'gps')
            [C, z] = gps_meaurement(llog(3), llog(4), first_gps, first_theta);
            % [C, z] = gps_meaurement_xy(llog(3), llog(4));
        elseif strcmp(name,'encoder')
            if (last_encoder == -1)
                last_encoder = llog(3);
            end
            [C, z] = encoder_meaurement(llog(3), time);
        elseif strcmp(name, 'steering')
            [C, z] = steering_measurement(llog(3));
        % elseif strcmp(name, 'imu_temp')
        %     [C, z] = compass_measurement(llog(3));
        else
            continue;
        end

        [P_pre, X_pre] = predict_step(P, X, time);
        [P, X] = update_step(C, z, P_pre, X_pre);
        snapshot = summarize(P, X, utm_zone);
        trajectory = [trajectory, snapshot];
    end

    if save_data
        save('localizer_v3.mat', 'trajectory', 'P');
    end
end

function snapshot = summarize(P, x, utm_zone)
    [lat, lon] = utm2ll(x(1), x(2), utm_zone);
    snapshot = [x; lat; lon; norm(P)];
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

% generate dynamic model
function [A] = model(x, time)
    global wheel_base
    global last_time

    dt = (time - last_time) / 1000.0;
    last_time = time;
    theta = deg2rad(x(3));
    heading = deg2rad(x(7));

    A = [1, 0, dt*cos(theta), -dt*sin(theta), 0, 0, 0;
         0, 1, dt*sin(theta), dt*cos(theta), 0, 0, 0;
         0, 0, 1, 0, 0, 0, 0;
         0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, dt, 0;
         0, 0, 180.0/(pi*(wheel_base/sin(heading))), 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0, 1];
end

function [P_pre, x_pre] = predict_step(P, x, time)
    R = eye(7); % measurement noise covariance
    A = model(x, time);
    x_pre = A * x;
    % P_pre = eye(7) * P * eye(7)'; % ERROR !!
    P_pre = A * P * A' + R;
    x_pre = scrubAngles(x_pre);
end

function [P, x] = update_step(C, z, P_pre, x_pre)
    Q = eye(7); % model noise covariance
    residual = z - (C * x_pre);
    residual = scrubAngles(residual);
    K = P_pre * C' * inv((C * P_pre * C') + Q); % gain
    x = x_pre + (K * residual);
    % P = (eye(7) - (K * C)); % ERROR !!
    P = (eye(7) - (K * C)) * P_pre;
end

function [C, z] = gps_meaurement(lat, long, first_gps, first_theta)
    global last_gps

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

function [C, z] = gps_meaurement_xy(lat, long)
    [x, y, ~] = ll2utm(lat, long);
    C = zeros(7, 7);
    C(1, 1) = 1;
    C(2, 2) = 1;
    z = [x, y, 0, 0, 0, 0 , 0]';
end

% TODO possible precision errors since changes are so small
% maybe batch encoder updates until we get a decent margin
function [C, z] = encoder_meaurement(encoder_dist, encoder_time)
    global last_encoder
    global last_encoder_time

    dx = encoder_dist - last_encoder;
    dt = encoder_time - last_encoder_time;
    C = zeros(7, 7);
    z = zeros(7, 1);
    
    if (dt > 1)
        last_encoder = encoder_dist;
        last_encoder_time = encoder_time;
        C(3, 3) = 1;
        z(3) = dx / (dt / 1000.0);
    end
end

function [C, z] = steering_measurement(heading)
    C = zeros(7, 7);
    C(7, 7) = 1;
    z = zeros(7, 1);
    z(7) = heading;
end

function [C, z] = imu_ang_pos_measurement(R)
    R = reshape(R, 3,3)';
    x = R(1, 1);
    x = R(1, 2);
    theta = 90 - rad2deg(atan2(y, x));

    C = zeros(7, 7);
    C(5, 5) = 1;
    z = zeros(7, 1);
    z(5) = theta;
end

% TODO add dtheta
function [C, z] = compass_measurement(c_angle)
    C = zeros(7, 7);
    C(5, 5) = 1;
    z = zeros(7, 1);
    z(5) = c_angle;
end


