function traj = make_trajectory(type, params)
    t0 = params.t(1);
    tf = params.t(2);
    q0 = params.q(1);
    qf = params.q(2);
    v0 = params.v(1);
    vf = params.v(2);
    dt = params.time_step;

    traj.t = t0:dt:tf;
    traj.q = zeros(size(traj.t));
    traj.v = zeros(size(traj.t));
    traj.a = zeros(size(traj.t));
    
    if strcmp('cubic', type)
        mat = [1 t0 t0^2 t0^3;
               0 1 2*t0 3*t0^2;
               1 tf tf^2 tf^3;
               0 1 2*tf 3*tf^2];
        
        b = mat \ [q0; v0; qf; vf];
        
        for i = 1:size(traj.t, 2)
            t = traj.t(i);
            
            traj.q(i) = b(1) + t*b(2) + t^2*b(3) + t^3*b(4);
            traj.v(i) = b(2) + 2*t*b(3) + 3*t^2*b(4);
            traj.a(i) = 2*b(3) + 6*t*b(4);
        end
    else
        a0 = params.a(1);
        af = params.a(2);
        
        mat = [1 t0 t0^2 t0^3 t0^4 t0^5;
               0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
               0 0 2 6*t0 12*t0^2 20*t0^3;
               1 tf tf^2 tf^3 tf^4 tf^5;
               0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
               0 0 2 6*tf 12*tf^2 20*tf^3];
        
        b = mat \ [q0; v0; a0; qf; vf; af];
        
        for i = 1:size(traj.t, 2)
            t = traj.t(i);
            
            traj.q(i) = b(1) + t*b(2) + t^2*b(3) + t^3*b(4) + t^4*b(5) + t^5*b(6);
            traj.v(i) = b(2) + 2*t*b(3) + 3*t^2*b(4) + 4*t^3*b(5) + 5*t^4*b(6);
            traj.a(i) = 2*b(3) + 6*t*b(4) + 12*t^2*b(5) + 20*t^3*b(6);
        end
    end
end
