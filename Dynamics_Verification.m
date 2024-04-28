%% Initialization
clear, clc, close all
addpath('lib');

plotOn = true;
nTests = 20; % number of random test configurations

%% Load the robot
robot = make_robot();
[S,M] = make_kinematics_model(robot);
[Mlist,Glist] = make_dynamics_model(robot);
g = [0; 0; -9.81];

% Joint limits
qlim = robot.qlim;

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.teach(q);
%% Set the friction forces and gear ratios to zero, as our rne doesn't take that into account
for i = 1:size(S,2)
    robot.links(i).G = 0;
    robot.links(i).Tc = 0;
    robot.links(i).B = 0;
    robot.links(i).Jm = 0;
end
%% Verify RNE algorithm
params_rne.g = g; % gravity
params_rne.S = S; % screw axes
params_rne.M = Mlist; % link frames
params_rne.G = Glist; % inertial properties
payload = 2;
fprintf('---------------------Inverse Dynamics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 20 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(),...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(),...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    params_rne.jointPos = q';
    params_rne.jointVel = [0.2;0.2;0.2;0.2;0.2;0.2];
    params_rne.jointAcc = [0.1;0.1;0.1;0.1;0.1;0.1];
    params_rne.Ftip = [1;3;2;0;0;-9.81];
    
    % end effector wrench (For toolbox RNE, the order is [force torque])
    Ftip_toolbox =  [params_rne.Ftip(4:6); params_rne.Ftip(1:3)]; 
    
    % Calculate the inverse dynamics using our RNE
    [tau, V, Vdot] = rne(params_rne);
    
    % Calculate the inverse dynamics using toolbox RNE
    tau_toolbox = robot.rne(q, params_rne.jointVel', params_rne.jointAcc', 'gravity', -g', 'fext', Ftip_toolbox' );
    if plotOn
        robot.teach(q);
        title('Inverse Dynamics Test');
    end
    
    %Verify the correctness of our RNE against toolbox RNE
    assert(all(all(abs(tau_toolbox - tau') < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');