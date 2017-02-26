function [trajectory] = controller_stanley()
% https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
% section 2.3

    addpath('../localizer/latlonutm/Codes/matlab');
    global wheel_base
    global velocity
    global steering_vel
    global dt
    global last_closest_idx
    global last_u

    save_data = true;
    wheel_base = 1.13;
    utm_zone = 17;
    first_heading = deg2rad(250);
    lat_long = [40.441670, -79.9416362]; % tri [40.442867, -79.9427395];
    dt = 0.001; % 1000Hz
    m = 50; % 20Hz
    velocity = 3.6; % m/s, 8mph, forward velocity
    steering_vel = deg2rad(40); % 40deg/s, reaction speed to control cmds
                                % full range in 0.5s
    last_closest_idx = 1;
    last_u = 0;
    total_time = 6; % min

    [x, y, ~] = ll2utm(lat_long(1), lat_long(2));

    X = [x;  % X, m, UTM coors
         y;  % Y, m, UTM coors
         velocity;  % d_Yb, body velocity
         first_heading; % heading, rad, world frame
         0];  % d_heading, rad/s

    load('./waypoints_course_v2.mat');
    desired_traj = processWaypoints(logs);
    desired_traj = desired_traj(112:(end-50), :);

    time = 0:dt:(total_time*60);
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

        snapshot = summarize(X, utm_zone, steering);
        trajectory = [trajectory, snapshot];
    end

    if save_data
        save('controller_v2.mat', 'trajectory');
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
    b = deg2rad(10);
    if(a < -b)
        a = -b;
    elseif(a > b)
        a = b;
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
    global wheel_base
    global last_closest_idx
    global last_u

    K = 0.1;
    K2 = 0.8;
    pos = X(1:2)';

    closest_idx = last_closest_idx;
    min_dist = 100000;
    for k=last_closest_idx:length(desired_traj)
        distp = norm(desired_traj(k,:) - pos);
        if(distp < min_dist)
            min_dist = distp;
            closest_idx = k;
        end
        % cut off search somehow
    end

    if(closest_idx == length(desired_traj))
        u = 0;
        return;
    end

    ptA = desired_traj(closest_idx, :);
    ptB = desired_traj(closest_idx+1, :);
    
    p = ptB - ptA; % path
    path_heading = atan2(p(2), p(1));
    heading_error = clampAngle(path_heading) - clampAngle(X(4));
    crosstrack_error = - det([p; pos - ptA]) / norm(p);

    u = K2*heading_error + atan2(K * crosstrack_error, X(3));

    % try moving average
    % temp_u = u;
    % u = (last_u + u) / 2;
    % last_u = temp_u;

    u = clampSteeringAngle(u);
    last_closest_idx = closest_idx;
end

