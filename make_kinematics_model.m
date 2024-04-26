function [S,M] = make_kinematics_model(robot)
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% the Panda 560 robot.
%
% Inputs: robot - the robot object created by the robotics toolbox
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

% Screw Axes
L1 = 0.2435;
L2 = 0.4318;
L3 = 0.4331;
offset1 = 0.0934;
offset2 = 0.0203;
w1 = [0 0 1]'; v1 = [0 0 0]';
w2 = [0 1 0]'; v2 = [0 0 0]';
w3 = [0 1 0]'; p3 = [L2 0 0]'; v3 = -cross(w3,p3);
w4 = [0 0 1]'; p4 = [L2 - offset2 L1-offset1 0]'; v4 = -cross(w4,p4);
w5 = [0 1 0]'; p5 = [L2 - offset2 0 L3]'; v5 = -cross(w5,p5);
w6 = w4; v6 = v4;
S = [w1 w2 w3 w4 w5 w6;
     v1 v2 v3 v4 v5 v6];

% Home configuration
M = double(robot.fkine(zeros(1,6)));

end