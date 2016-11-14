function [trajectory] = controller()

    addpath('../localizer/latlonutm/Codes/matlab');
    global wheel_base



end

function [A] = model(x, u, dt)
    global wheel_base

    A = [1, 0, dt*cos(x(3)), 0, 0;
         0, 1, dt*sin(x(3)), 0, 0;
         0, 0, 1, 0, 0;
         0, 0, 0, 1, dt;
         0, 0, tan(u)/wheel_base, 0, 0];
end


