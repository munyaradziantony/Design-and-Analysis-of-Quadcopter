function output = looptrajectory(input)
    persistent state stime xd yd zd otime
    x     = round(input(1),2);
    y     = round(input(2),2);
    z     = round(input(3),2);
    sim_t = input(4);

    if (sim_t == 0 )
        state = 0;
        stime = sim_t;
        xd = 3;
        yd = 5;
        zd = 6;
    end

    if (state == 0 && x == 3 && y == 5 && z == 6)
        otime = sim_t-stime;
        if (otime <= 0.1)
            xd = -7;
            yd = 10;
            zd = 4;
            state = 1;
        end
        stime = sim_t; % update time
    end

    if (state == 1 && x == -7 && y == 10 && z == 4)
        otime = sim_t-stime;
        if (otime <= 0.1)
            xd = 0;
            yd = 0;
            zd = 0;
            state = 2;
        end
        stime = sim_t; % update time
    end

    if (state == 2 && x ==0 && y == 0 && z == 0)
        otime = sim_t-stime;
        if (otime <= 0.1)
            xd = 3;
            yd = 5;
            zd = 6;
            state = 0;
        end
        stime = sim_t; % update time
    end

    % Desired 0 degree in yaw 
    psid    = 0;
    psidotd = 0;
    output = [xd; yd; zd; psid; psidotd; sim_t]; % psid and psidotd are set to 0
end