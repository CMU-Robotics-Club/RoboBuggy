function [trajectory] = controller()

    addpath('../localizer/latlonutm/Codes/matlab');
    global wheel_base
    global velocity

    save_data = true;
    wheel_base = 1.13;
    utm_zone = 17;
    first_heading = deg2rad(250);
    lat_long = [40.441670, -79.9416362];
    dt = 0.001; % 1000Hz
    m = 20; % 20Hz
    velocity = 3; % m/s, 6.7mph

    [x, y, ~] = ll2utm(lat_long(1), lat_long(2));

    X = [x;  % X, m, UTM coors
         y;  % Y, m, UTM coors
         velocity;  % d_Yb, body velocity
         first_heading; % heading, rad, world frame
         0];  % d_heading, rad/s

    load('./waypoints.mat');
    desired_traj = processWaypoints(logs);
    % time = linspace(0, 240, size(trajectory,2));
    time = 0:dt:240;
    u = 0; % steering angle
    trajectory = [X; lat_long'; u];

    for i = 1:size(time, 2)
        t = time(i);
        A = model(X, u, dt);
        X = A*X;

        X(4) = clampAngle(X(4));
        X(5) = clampAngle(X(5));

        if(mod(i, m) == 0)
            u = control(desired_traj, X);
            u = clampAngle(u);
        end

        % trajectory = [trajectory, X];
        snapshot = summarize(X, utm_zone, u);
        trajectory = [trajectory, snapshot];
    end
    
    if save_data
        save('controller_v1.mat', 'trajectory');
    end
end

function [desired] = processWaypoints(lat_long)
    [x, y, zone] = ll2utm(lat_long);
    desired = [x y];
end

function snapshot = summarize(x, utm_zone, steeringAngle)
    [lat, lon] = utm2ll(x(1), x(2), utm_zone);
    snapshot = [x; lat; lon; steeringAngle];
end

function a = clampAngle(a)
    while (a < -pi)
        a = a + 2*pi;
    end
    while (a > pi)
        a = a - 2*pi;
    end
end

function [A] = model(x, u, dt)
    global wheel_base

    A = [1, 0, dt*cos(x(3)), 0, 0;
         0, 1, dt*sin(x(3)), 0, 0;
         0, 0, 1, 0, 0;
         0, 0, 0, 1, dt;
         0, 0, tan(u)/wheel_base, 0, 0];
end

function [u] = control(desired_traj, X) 
    pos = X(1:2)';
    b = repmat(pos, size(desired_traj, 1), 1);
    delta = 100;
    possible = find(sum((desired_traj-b).^2, 2) < delta);
    if isempty(possible)
      u = pi;
    else
      target = desired_traj(possible(end), :);
      deltaPath = target - pos;
      u = atan2(target(2), target(1));
    end
end



