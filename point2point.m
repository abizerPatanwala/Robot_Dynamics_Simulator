%% Load the robot
clear, clc, close all
addpath('lib');

plotOn = true;

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot and display it in the home configuration
robot = make_robot();
% robot.teach(zeros(1,6));

% Create a kinematic model of the robot
[S,M] = make_kinematics_model(robot);
n = size(S,2); % read the number of joints

% Create a dynamical model of the robot
[Mlist,Glist] = make_dynamics_model(robot);

%% Set the friction forces and gear ratios to zero, as our rne doesn't take that into account
for i = 1:size(S,2)
    robot.links(i).G = 0;
    robot.links(i).Tc = 0;
    robot.links(i).B = 0;
    robot.links(i).Jm = 0;
end
%% Define the waypoints in task space
% path = [0 0.3 0.15;
%         0.25 0 0.1; ...
%         0.25 0.25 0.2]';
path = [0.644 0.150 -0.027;
        0.367 0.550 -0.027; ...
        0.015 0.661 -0.027]';

%% Define Inverse Dynamics Tracking parameter
dt = 1e-03;
Total_Time = 5;
payload = 1;

%% Calculate IK for the given waypoints
fprintf('----------------------Ik Calculation--------------------\n');
fprintf('Calculating the Inverse Kinematics... ');
robot.plot(zeros(1,6)); hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('Inverse Dynamics Control');

% Calculate the inverse kinematics
waypoints = ikin(S, M, path);
fprintf(' IK Done.\n');

%% Calculate the trajectories for the given points
fprintf('----------------------Trajectory Generation--------------------\n');
fprintf('Calculating the Trajectories... ');
nbytes = fprintf('0%%');
nPts = size(path,2);

% Initialize the time vector
t  = 0 : dt : Total_Time; % total time [s]

% Initialize the array for storing Prescribed joint states and acceleration
jointPos_prescribed = zeros(n,size(t,2), nPts -1); % Joint Variables (Prescribed)
jointVel_prescribed = zeros(n,size(t,2), nPts -1); % Joint Velocities (Prescribed)
jointAcc_prescribed = zeros(n,size(t,2), nPts -1); % Joint Accelerations (Prescribed)

for jj = 1 : nPts - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
  
    % For each joint
    for ii = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.time_step = dt;
        params_traj.q = [waypoints(ii,jj) waypoints(ii,jj+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(ii,:, jj) = traj.q;
        jointVel_prescribed(ii,:, jj) = traj.v;
        jointAcc_prescribed(ii,:, jj) = traj.a;
    end
end
fprintf(' Trajectory Generation Done.\n');
%% Control the motion of the robot between 2 set points
fprintf('----------------------Dynamic Control of a 3-DoF Arm--------------------\n');

nPts = size(path,2);
% Now, for each configuration in the prescribe trajectory we will calculate the torque profile
fprintf('Generating the Torque Profiles... ');
nbytes = fprintf('0%%');

% Inititalize the variables where we will store the actual torque profiles, joint
% positions, and time, so that we can display them later
tau_acc = [];
jointPos_acc = [];
t_acc = [];
jointAcc_Matrix = [];
Ftip = [];
% Initialize the parameters for both inverse and forward dynamics


for jj = 1 : nPts - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
    %fprintf('\n Point is %d', jj);
    % Initialize the time vector
    t  = 0 : dt : Total_Time; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    tau_prescribed = zeros(n,size(t,2)); % Joint Torques
    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)

    % Initialize the (actual) joint variables
    jointPos_actual(:,1) = jointPos_prescribed(:,1, jj);
    jointVel_actual(:,1) = jointVel_actual(:,1);
    
    for ii = 1 : size(t,2) - 1
        params_rne.g = g'; % gravity
        params_rne.S = S; % screw axes
        params_rne.M = Mlist; % link frames
        params_rne.G = Glist; % inertial properties
        params_fdyn.g = g; % gravity
        params_fdyn.S = S; % screw axes
        params_fdyn.M = Mlist; % link frames
        params_fdyn.G = Glist; % inertial properties
       
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:,ii, jj);
        params_rne.jointVel = jointVel_prescribed(:,ii, jj);
        params_rne.jointAcc = jointAcc_prescribed(:,ii, jj);
        params_rne.Ftip = zeros(6,1); % end effector wrench
        % T = fkine(S, M, jointPos_prescribed(:,ii, jj), 'space');
        % params_rne.Ftip = AdjointMatrix(T)'*[payload*cross(T(1:3,4), -g'); -payload*g'];
        tau_prescribed(:,ii) = rne(params_rne);
        Ftip = [Ftip params_rne.Ftip];
        %fprintf('\n %d Point Calculation done', ii); 
        
        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        params_fdyn.Ftip = zeros(6,1); % end effector wrench
        % T = fkine(S, M, jointPos_actual(:,ii), 'space');
        % params_fdyn.Ftip =  AdjointMatrix(T)'*[payload*cross(T(1:3,4), -g'); -payload*g'];
        
        jointAcc = fdyn(params_fdyn);
        
        jointAcc_Matrix = [jointAcc_Matrix jointAcc];
        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
        jointPos_actual(:,ii+1) = wrapToPi(jointPos_actual(:,ii+1));
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    jointAcc_Matrix = [jointAcc_Matrix jointAcc];
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t+t(end)*(jj-1)];
end

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
title('Inverse Dynamics Control');
robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2}, 'movie', 'RBE-501-2023-HW5-point2point.mp4');
fprintf('Done.\n');


%% Display the Joint Torques
figure(1), hold on, grid on
plot(t_acc, tau_acc(1,:), 'Linewidth', 2);
plot(t_acc, tau_acc(2,:), 'Linewidth', 2);
plot(t_acc, tau_acc(3,:), 'Linewidth', 2);
plot(t_acc, tau_acc(4,:), 'Linewidth', 2);
plot(t_acc, tau_acc(5,:), 'Linewidth', 2);
plot(t_acc, tau_acc(6,:), 'Linewidth', 2);
title('Torque Profiles');
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
set(gca, 'FontSize', 14);

figure(2), hold on, grid on
plot(t_acc, jointAcc_Matrix, 'Linewidth', 2);
fprintf('Program completed successfully.\n');