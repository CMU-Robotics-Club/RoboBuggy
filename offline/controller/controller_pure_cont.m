function [trajectory] = controller_pure()
% https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
% section 2.2

    addpath('../localizer/latlonutm/Codes/matlab');
    global wheel_base
    global velocity
    global steering_vel
    global dt
    global last_closest_idx

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

    [x, y, ~] = ll2utm(lat_long(1), lat_long(2));

    X = [x;  % X, m, UTM coors
         y;  % Y, m, UTM coors
         velocity;  % d_Yb, body velocity
         first_heading; % heading, rad, world frame
         0];  % d_heading, rad/s

    load('./waypoints_course_v2.mat');
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
        save('controller_v3.mat', 'trajectory');
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

    k = 3;
    vel = X(3);
    pos = X(1:2)';
    lookahead_bounds = [3 25];
    lookahead = k * vel;
    if(lookahead < lookahead_bounds(1))
        lookahead = lookahead_bounds(1);
    elseif(lookahead > lookahead_bounds(2))
        lookahead = lookahead_bounds(1);
    end 

    % closest_idx = last_closest_idx;
    % min_dist = 100000;
    % for k=last_closest_idx:(length(desired_mid) - 1)
    %     distp = norm(desired_mid(k,:) - X);
    %     if(distp < min_dist)
    %         min_dist = distp;
    %         closest_idx = k;
    %     end
    %     % cut off search somehow
    % end

    % ptA = [0 0];
    % if(closest_idx == 1)
    %     ptA = X;
    % else
    %     ptA = desired_traj(closest_idx, :);
    % end
    % ptB = desired_traj(closest_idx+1, :);
    % crosstrack_error = abs(det([ptB - ptA; X - ptA])) / norm(ptB - ptA);

    closest_idx = last_closest_idx;
    min_dist = 100000;
    for k=last_closest_idx:length(desired_traj)
        distp = norm(desired_traj(k,:) - X);
        if(distp < min_dist)
            min_dist = distp;
            closest_idx = k;
        end
        % cut off search somehow
    end

    lookahead_idx = 0;
    for k=closest_idx:length(desired_traj)
        distp = norm(desired_traj(k,:) - X);
        if(distp > lookahead)
            lookahead_idx = k;
            break;
        end
    end

    if(lookahead_idx == 0)
        u = 0;
    else
        deltaPath = desired_traj(lookahead_idx,:) - pos;
        a = atan2(deltaPath(2), deltaPath(1)) - (pi/2);
        u = atan2(2 * wheel_base * sin(a), lookahead);
    end

    % maybe consider deadband region

    last_closest_idx = closest_idx;
    u = clampSteeringAngle(u);
end

