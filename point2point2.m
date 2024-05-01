function [t_acc, jointPos_acc, jointVel_acc, jointAcc_acc, tau_acc, valid] = point2point2(path, Forces, init_point)
%% Load the robot
addpath('lib');
% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot and display it in the home configuration
robot = make_robot();

% Create a kinematic model of the robot
[S,M] = make_kinematics_model(robot);
n = size(S,2); % read the number of joints

% Create a dynamical model of the robot
[Mlist,Glist] = make_dynamics_model(robot);

%% Define Inverse Dynamics Tracking parameter
dt = 1e-03;
Total_Time = 0.5;

%% Calculate IK for the given waypoints
% Calculate the inverse kinematics
T = eye(4);
T(1:3,4) = path(1:3);
T(1:3,1:3) = rotz(path(6))*roty(path(5))*rotx(path(4));
T_Log = MatrixLog6(T);
V_Des = [T_Log(3,2) T_Log(1,3) T_Log(2,1) T_Log(1:3,4)']'; 
qinit = [pi/3 pi/4 pi/6 pi/3 pi/4 pi/6; 0 0 0 0 0 0; 0 pi/2 0 0 0 0; pi/2 0 0 0 0 0; init_point'];
waypoints = zeros(n,2);
waypoints(:,1) = init_point;
valid = false;
for i = 1:size(qinit,1)
    curr_sol = robot.ikine(T, 'q0', qinit(i,:), 'tol', 1e-03, 'ilimit', 2000, 'quiet');
    if ~isempty(curr_sol)
        waypoints(:,2) = curr_sol';
        valid = true;
        break;
    else
        continue;
    end
end

if valid == true
%% Calculate the trajectories for the given points
nPts = size(waypoints,2);

% Initialize the time vector
t  = 0 : dt : Total_Time; % total time [s]

% Initialize the array for storing Prescribed joint states and acceleration
jointPos_prescribed = zeros(n,size(t,2), nPts -1); % Joint Variables (Prescribed)
jointVel_prescribed = zeros(n,size(t,2), nPts -1); % Joint Velocities (Prescribed)
jointAcc_prescribed = zeros(n,size(t,2), nPts -1); % Joint Accelerations (Prescribed)

for jj = 1 : nPts - 1
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
%% Control the motion of the robot between 2 set points
nPts = size(waypoints,2);
% Now, for each configuration in the prescribe trajectory we will calculate the torque profile
% Inititalize the variables where we will store the actual torque profiles, joint
% positions, and time, so that we can display them later
tau_acc = [];
jointPos_acc = [];
jointVel_acc = [];
jointAcc_acc = [];
t_acc = [];
MM_Mat = [];

for jj = 1 : nPts - 1
    % Initialize the time vector
    t  = 0 : dt : Total_Time; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    tau_prescribed = zeros(n,size(t,2)); % Joint Torques
    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)
    jointAcc_actual = zeros(n,size(t,2)); % Joint Accelerations (Actual)

    % Initialize the (actual) joint variables
    jointPos_actual(:,1) = jointPos_prescribed(:,1, jj);
    jointVel_actual(:,1) = jointVel_actual(:,1);
    
    for ii = 1 : size(t,2) - 1
        params_rne.g = g'; % gravity
        params_rne.S = S; % screw axes
        params_rne.M = Mlist; % link frames
        params_rne.G = Glist; % inertial properties
        params_fdyn.g = g'; % gravity
        params_fdyn.S = S; % screw axes
        params_fdyn.M = Mlist; % link frames
        params_fdyn.G = Glist; % inertial properties
       
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:,ii, jj);
        params_rne.jointVel = jointVel_prescribed(:,ii, jj);
        params_rne.jointAcc = jointAcc_prescribed(:,ii, jj);
        params_rne.Ftip = zeros(6,1); % end effector wrench
        T = fkine(S, M, jointPos_prescribed(:,ii, jj), 'space');
        params_rne.Ftip = AdjointMatrix(T)'*[cross(T(1:3,4), -Forces); -Forces];
        tau_prescribed(:,ii) = rne(params_rne); 
        
        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        params_fdyn.Ftip = zeros(6,1); % end effector wrench
        T = fkine(S, M, jointPos_actual(:,ii), 'space');
        params_fdyn.Ftip = AdjointMatrix(T)'*[cross(T(1:3,4), -Forces); -Forces]; 
        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
        jointAcc_actual(:, ii+1) = jointAcc;
    end
    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    jointVel_acc = [jointVel_acc jointVel_actual];
    jointAcc_acc = [jointAcc_acc jointAcc_actual];
    t_acc = [t_acc t+t(end)*(jj-1)];
end
else
    t_acc = [];
    jointPos_acc = [];
    jointVel_acc = [];
    jointAcc_acc = [];
    tau_acc = [];
end
end