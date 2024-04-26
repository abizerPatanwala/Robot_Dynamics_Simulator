function R = axisangle2rot(omega,theta)
    skew_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    R = eye(3) + sin(theta)*skew_omega + (1 - cos(theta))*(skew_omega*skew_omega);
end