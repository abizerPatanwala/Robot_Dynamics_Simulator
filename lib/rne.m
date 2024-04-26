function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
% YOUR CODE HERE 
Joints = size(params.S,2);
V = zeros(6,Joints + 1);
Vdot = zeros(6,Joints + 1);
tau = zeros(Joints,1);
Vdot(1:6, 1) = [0 0 0 -params.g(1) -params.g(2) -params.g(3)]';
A = zeros(6,Joints);
for i = 1 : Joints
    Mc = eye(4);
    for j = 1:i
        Mc = Mc * params.M(:,:,j); 
    end
    Ac = adjoint(pinv(Mc)) * params.S(:,i);
    Mcp = pinv(params.M(:,:,i));
    Tcp = twist2ht(-Ac, params.jointPos(i)) * Mcp;
    V(:,i+1) =  Ac*params.jointVel(i) + adjoint(Tcp)*V(:,i); % Link Velocity
    Vdot(:,i+1) = Ac*params.jointAcc(i) + adjoint(Tcp)* Vdot(:,i) + ad(V(:,i+1))*Ac*params.jointVel(i); % Link Acceleration
    A(:,i) = Ac;
end
% Backward iterations
W = zeros(6,Joints+1);
W(1:6,Joints+1) = params.Ftip;
for i = Joints: -1: 1
    Gc = params.G(:,:,i);
    Vc = V(:,i+1);
    Vcdot = Vdot(:,i+1);
    if i == Joints
        Tnc = pinv(params.M(:,:,i+1));
    else
        An = A(1:6,i+1); 
        Tnc = twist2ht(-An, params.jointPos(i+1)) * pinv(params.M(:,:,i+1));
    end
    W(:,i) = Gc*Vcdot - (ad(Vc)')*Gc*Vc + adjoint(Tnc)'*W(:,i+1);
    tau(i) = W(:,i)'*A(:,i);
end
% YOUR CODE HERE

end
function AdT = adjoint(T)
    % your code here
    R = T(1:3,1:3);
    p = T(1:3,4);
    skew_p = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    AdT = [R zeros(3,3); skew_p*R R];
end
function adV = ad(V)
    % your code here
    w = V(1:3);
    v = V(4:6);
    skew_w = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
    skew_v = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    adV = [skew_w zeros(3); skew_v skew_w];
end
function T = twist2ht(S,theta)
    omega = S(1:3);
    skew_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    v = S(4:6);
    R = axisangle2rot(omega,theta);
    T = [R (eye(3)*theta + (1 - cos(theta))*skew_omega + (theta - sin(theta))*(skew_omega*skew_omega))*v; 0 0 0 1]; 
end
function R = axisangle2rot(omega,theta)
    skew_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    R = eye(3) + sin(theta)*skew_omega + (1 - cos(theta))*(skew_omega*skew_omega);
end