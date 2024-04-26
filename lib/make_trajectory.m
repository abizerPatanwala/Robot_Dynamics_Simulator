function traj = make_trajectory(type, params)
% YOUR CODE HERE
     t0 = params.t(1);
     tf = params.t(2);
     q0 = params.q(1);
     qf = params.q(2);
     qdot0 = params.v(1);
     qdotf = params.v(2);
     timestep = params.time_step;
    if strcmp(type, 'cubic')
       T = [1, t0, t0^2, t0^3;
            0, 1, 2*t0, 3*t0^2;
            1, tf, tf^2, tf^3;
            0, 1, 2*tf, 3*tf^2];
       coeffs = inv(T)*[q0; qdot0; qf; qdotf];
       traj.t = t0:timestep:tf;
       traj.q = coeffs(1) + coeffs(2)*traj.t + coeffs(3)*traj.t.^2 + coeffs(4)*traj.t.^3;
       traj.v = coeffs(2) + 2*coeffs(3)*traj.t + 3*coeffs(4)*traj.t.^2;
       traj.a = 2*coeffs(3) + 6*coeffs(4)*traj.t;

    elseif(strcmp(type, 'quintic'))
       qddot0 = params.a(1);
       qddotf = params.a(2);
       T = [1, t0, t0^2, t0^3, t0^4, t0^5;
            0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
            0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
            1, tf, tf^2, tf^3, tf^4, tf^5;
            0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
            0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
       coeffs = inv(T)*[q0; qdot0; qddot0; qf; qdotf; qddotf];
       traj.t = t0:timestep:tf;
       traj.q = coeffs(1) + coeffs(2)*traj.t + coeffs(3)*traj.t.^2 + coeffs(4)*traj.t.^3  + coeffs(5)*traj.t.^4 + coeffs(6)*traj.t.^5;
       traj.v = coeffs(2) + 2*coeffs(3)*traj.t + 3*coeffs(4)*traj.t.^2 + 4*coeffs(5)*traj.t.^3 + 5*coeffs(6)*traj.t.^4;
       traj.a = 2*coeffs(3) + 6*coeffs(4)*traj.t + 12*coeffs(5)*traj.t.^2 + 20*coeffs(6)*traj.t.^3;
    end 
end