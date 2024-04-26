function T = twist2ht(S,theta)
    omega = S(1:3);
    skew_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    v = S(4:6);
    R = axisangle2rot(omega,theta);
    T = [R (eye(3)*theta + (1 - cos(theta))*skew_omega + (theta - sin(theta))*(skew_omega*skew_omega))*v; 0 0 0 1]; 
end