function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_DYNAMICS_MODEL Creates the dynamics model of the Puma 560
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Link poses when the robot is in the home configuration
[M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
J1 = diag([0, 0, 0.35]);
J2 = diag([0.13, 0.524, 0.539]);
J3 = diag([0.066, 0.0125, 0.066]);
J4 = diag([1.8e-3, 1.8e-3, 1.3e-3]);
J5 = diag([0.3e-3, 0.3e-3, 0.4e-3 ]);
J6 = diag([0.15e-3, 0.15e-3, 0.04e-3]);

m1 = 0;
m2 = 17.4;
m3 = 4.8;
m4 = 0.82;
m5 = 0.34;
m6 = 0.09;

G1 = [J1 zeros(3,3); zeros(3,3) m1*eye(3)];
G2 = [J2 zeros(3,3); zeros(3,3) m2*eye(3)];
G3 = [J3 zeros(3,3); zeros(3,3) m3*eye(3)];
G4 = [J4 zeros(3,3); zeros(3,3) m4*eye(3)];
G5 = [J5 zeros(3,3); zeros(3,3) m5*eye(3)];
G6 = [J6 zeros(3,3); zeros(3,3) m6*eye(3)];
Glist = cat(3, G1, G2, G3, G4, G5, G6);

end