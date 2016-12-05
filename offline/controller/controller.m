function [trajectory] = controller()

    addpath('../localizer/latlonutm/Codes/matlab');
    global wheel_base
    global velocity
    global steering_vel
    global dt

    save_data = true;
    wheel_base = 1.13;
    utm_zone = 17;
    first_heading = deg2rad(250);
    lat_long = [40.441670, -79.9416362];
    dt = 0.001; % 1000Hz
    m = 50; % 20Hz
    velocity = 8; % m/s, 17.9mph, forward velocity
    steering_vel = deg2rad(40); % 40deg/s, reaction speed to control cmds

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
    u = 0; % commanded steering angle
    steering = u; % steering angle
    trajectory = [X; lat_long(1); lat_long(2); steering];

    for i = 1:size(time, 2)
        t = time(i);
        A = model(X, steering);
        X = A*X;
        steering = updateSteering(steering, u);

        X(4) = clampAngle(X(4));
        X(5) = clampAngle(X(5));

        if(mod(i, m) == 0)
            u = control(desired_traj, X);
        end

        % trajectory = [trajectory, X];
        snapshot = summarize(X, utm_zone, steering);
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
    snapshot = [x; x(1); x(2); steeringAngle];
end

function a = clampAngle(a)
    while (a < -pi)
        a = a + 2*pi;
    end
    while (a > pi)
        a = a - 2*pi;
    end
end

function b = updateSteering(b, u)
    global steering_vel
    global dt

    if(b < u)
        b = b + steering_vel*dt;
    end
    if(b > u)
        b = b - steering_vel*dt;
    end
end

function a = clampSteeringAngle(a)
    if(a < -deg2rad(10))
        a = -deg2rad(10);
    end
    if(a > deg2rad(10))
        a = deg2rad(10);
    end
end

function [A] = model(x, steering)
    global wheel_base
    global dt

    A = [1, 0, dt*cos(x(4)), 0, 0;
         0, 1, dt*sin(x(4)), 0, 0;
         0, 0, 1, 0, 0;
         0, 0, 0, 1, dt;
         0, 0, tan(steering)/wheel_base, 0, 0];
end

function [u] = control(desired_traj, X) 
    pos = X(1:2)';
    b = repmat(pos, size(desired_traj, 1), 1);
    delta = 15*15;
    possible = find(sum((desired_traj-b).^2, 2) < delta);
    if isempty(possible)
      u = 0;
    else 
      target = desired_traj(possible(end), :);
      deltaPath = target - pos;
      u = atan2(deltaPath(2), deltaPath(1))-X(4);
    end
    u = clampSteeringAngle(clampAngle(u));
end



