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
    global R
    global steeringAngle
    global first_gps
    global first_theta
    save_data = true;

    % constants
    ref_dt = (1/10) * 1000; % 10 Hz in ms
    % dt of the controller, not the filter
    wheel_base = 1.13;
    utm_zone = 17;
    first_theta = deg2rad(250);
    lat_long = [40.441670, -79.9416362];

    [x, y, zone] = ll2utm(lat_long(1), lat_long(2));
    first_gps = [x y];
    last_gps = [x y];
    utm_zone = zone; % fixed
    last_encoder = 0;
    steeringAngle = 0;

    % output matrices
    C_encoder = [0 0 1 0 0];
    C_gps = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 0 1 0];

    % measurement noise covariance
    R = diag([4, 4, 0.25, 0.01, 0.01]);
    % model noise covariance
    Q_gps = diag([4, 4, 0.02]);
    Q_encoder = [0.25];
    % initialize Kalman filter
    % covariance matrix
    P = diag([25, 25, 0.25, 2.46, 2.46]);

    X = [x;  % X, m, UTM coors
         y;  % Y, m, UTM coors
         0;  % d_Yb, body velocity, m/s
         first_theta; % heading, deg, global
         0];  % d_heading, deg/s

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
            [z] = gps_meaurement(llog(3), llog(4), X(4));
            C = C_gps;
            Q = Q_gps;
        elseif strcmp(name,'encoder')
            [z] = encoder_meaurement(llog(3), time);
            C = C_encoder;
            Q = Q_encoder;
            if isempty(z)
                continue;
            end
        elseif strcmp(name, 'steering')
            steeringAngle = deg2rad(llog(3));
            continue;
        else
            continue;
        end

        [P_pre, X_pre] = predict_step(P, X, time);
        [P, X] = update_step(C, Q, z, P_pre, X_pre);
        snapshot = summarize(P, X, utm_zone);
        trajectory = [trajectory, snapshot];
    end

    if save_data
        save('localizer_v4.mat', 'trajectory', 'P');
    end
end

function snapshot = summarize(P, x, utm_zone)
    [lat, lon] = utm2ll(x(1), x(2), utm_zone);
    snapshot = [x; lat; lon; norm(P)];
end

function a = clampAngle(a)
    while (a < -pi)
        a = a + 2*pi;
    end
    while (a > pi)
        a = a - 2*pi;
    end
end

% generate dynamic model
function [A] = model(x, time)
    global wheel_base
    global last_time
    global steeringAngle

    dt = (time - last_time) / 1000.0;
    last_time = time;

    A = [1, 0, dt*cos(x(4)), 0, 0;
         0, 1, dt*sin(x(4)), 0, 0;
         0, 0, 1, 0, 0;
         0, 0, 0, 1, dt;
         0, 0, tan(steeringAngle)/wheel_base, 0, 0];
end

function [P_pre, x_pre] = predict_step(P, x, time)
    global R
    A = model(x, time);
    x_pre = A * x;
    P_pre = (A * P * A') + R;
    
    x_pre(4) = clampAngle(x_pre(4));
    x_pre(5) = clampAngle(x_pre(5));
end

function [P, x] = update_step(C, Q, z, P_pre, x_pre)
    residual = z - (C * x_pre);
    K = P_pre * C' * inv((C * P_pre * C') + Q); % gain
    x = x_pre + (K * residual);
    P = (eye(5) - (K * C)) * P_pre;

    x(4) = clampAngle(x(4));
    x(5) = clampAngle(x(5));
end

function [z] = gps_meaurement(lat, long, last_heading)
    global last_gps
    global first_gps
    global first_theta

    [x, y, ~] = ll2utm(lat, long);
    dx = x - last_gps(1);
    dy = y - last_gps(2);
    last_gps = [x y];
    heading = atan2(dy, dx);
    
    % ignore small strides
    if ((dx*dx + dy*dy) < 0.25)
        heading = last_heading;
    end
    % close the loop
    d = abs(x - first_gps(1)) + abs(y - first_gps(2));
    if (d < 10.0)
        heading = first_theta;
    end

    z = [x; y; heading];
end

function [z] = encoder_meaurement(encoder_dist, encoder_time)
    global last_encoder
    global last_encoder_time

    dt = encoder_time - last_encoder_time;
    
    if (dt < 10)
        z = [];
    else
        dx = encoder_dist - last_encoder;
        last_encoder = encoder_dist;
        last_encoder_time = encoder_time;
        bodySpeed = dx / (dt / 1000.0);
        z = [bodySpeed];
    end
end



