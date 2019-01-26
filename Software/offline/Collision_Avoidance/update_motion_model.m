function A = update_motion_model(x, delta, dt)
    
    A = [
        1, 0, cos(x(4))*dt, 0, 0;
        0, 1, sin(x(4))*dt, 0, 0;
        0, 0, 1, 0, 0;
        0, 0, 0, 1, dt;
        0, 0, tan(delta), 0, 0;
    ];

end