function relative_locs = get_relative_object_locs(x, obstacle_locs)

xloc = [ x(1) x(2) ];
num_obstacles = size(obstacle_locs, 1);

% get the world frame's relative locations
relative_locs_world_frame = obstacle_locs - ones(num_obstacles,1)*xloc;

% but we need the relative locations from the point of view from the
% buggy's frame
% see the docs.pdf on relative coordinates for the terminology
d = sum(relative_locs_world_frame .^ 2, 2);

psi = x(4);
theta = atan2(relative_locs_world_frame(:,2), relative_locs_world_frame(:,1));
phi = theta - ones(size(theta))*psi;

a = d .* cos(phi);
b = d .* sin(phi);

relative_locs = [ a b ];

end