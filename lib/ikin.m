function [q_sol] = ikin(S, M, qinit, V_des)

% Define Joint limits
% Define starting configuration of the robot
currentQ = qinit;

% Define current pose for the first iteration
T = fkine(S,M,currentQ, 'space');
currentPose = MatrixLog6(T);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Generate the robot's target pose
targetPose = V_des;

% Inverse Kinematics
lambda = 0.1;
iter = 1;
Sol_Found = true;
while norm(targetPose - currentPose) > 1e-03 
     if iter > 2000
         Sol_Found = false;
         break;
     end
    % Calculate jacobian of current configuration
    J = jacob0(S,currentQ);

    % Calculate deltaQ via LM algorithm
    deltaQ = J' * pinv(J*J' + lambda^2 * eye(6)) * (targetPose - currentPose);
    % deltaQ = pinv(J) * (targetPose - currentPose);
    
    % Update current joint variable 
    currentQ = currentQ + deltaQ';
    currentQ = wrapToPi(currentQ);

    % Calculate currentPose to be used in next iteration
    T = fkine(S,M,currentQ, 'space');
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
    iter = iter + 1;
end
if Sol_Found == true
    q_sol = currentQ;
else
    q_sol = [];
end