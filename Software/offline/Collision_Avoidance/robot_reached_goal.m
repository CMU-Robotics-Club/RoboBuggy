function b = robot_reached_goal(x, goal)

    xpair = [ x(1) x(2) ];
    d = sqrt(sum((xpair - goal) .^ 2));
    b = (d < 0.5);

end