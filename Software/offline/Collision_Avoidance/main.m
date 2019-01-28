% Main execution file
% Sets up the simulation environment and the robot, as well as all params

% start

% initialize simulation parameters
dt = 0.5;
figurenum_simwindow = 1;
prev_robot_locations = [
    0 0
];

% initialize robot
x = [
    0;
    0;
    1;
    0;
    0;
    ];
delta = 0;
A = update_motion_model(x, delta, dt);

% initialize obstacle and goal locations
obstacle_locs = [
    1 1;
    2 2;
    1 -1;
    -1 1;
    -1 -1;
];
goal_loc = [
    10, 0
];

% initialize GUI
update_sim_gui(figurenum_simwindow, obstacle_locs, goal_loc, prev_robot_locations, x);

% check if robot has reached the goal
% if it has, end the sim and freeze frame
while (~robot_reached_goal(x, goal_loc))

    % step the robot forward using the motion model
    A = update_motion_model(x, delta, dt);
    x = A * x;
    
    % update the robot's understanding of obstacles
    relative_locs = get_relative_object_locs(x, obstacle_locs);
    
    % run the robot control algo
    delta = avoid_collision(x, delta, relative_locs);
    
    % update the robot's parameters
    % already did it, set delta
    
    % update the GUI
    update_sim_gui(figurenum_simwindow, obstacle_locs, goal_loc, prev_robot_locations, x);

end