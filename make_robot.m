function robot = make_robot()
%MAKE_ROBOT Creates the kinematic structure of the robot used in homework 4, problem 2.
%
%   This is a factory function that creates the robot used in the homework.
%
%   Inputs: none
%
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox

% Create the manipulator
mdl_puma560akb;
robot = p560m;
% mdl_ur5;
% robot = ur5;
% qz = [0 0 0 0 0 0];
% robot.plot(qz)
% [T,A] = robot.fkine(qz);
% hold on
% for i=1:6
%   trplot(A(i), 'frame', num2str(i))
% end
% robot.plot(zeros(1,6), 'jaxes')
end

