
%% verify update frames
% Q = rand(7,1);
Q = ones(7,1)*pi; Q(7) = 0;
% Q = [0.0811,    0.9294,    0.7757,    0.4868,    0.4359,    0.4468,    0.3063];


RobotConfig = Set7DOFsParameters();
RobotFrames = UpdateAllFrames(Q,RobotConfig);

RobotJointFrames = UpdateJointFrames_MS7(Q);

% for i = 1:7, compare the transformation matrix
i = 2; eval(['[RobotJointFrames.T0_i(:,:,', num2str(i),')  zeros(4,2)      RobotFrames.AF_Base(:,:,', num2str(i),')]']);

RobotJointFrames.T0_i,
RobotJointFrames.T0_tool,

%% verify the gravity compensator model with JointFrame in MS7(without DH parameters).
Q = ones(7,1)*pi; 
Q(2) = pi/2;
GravityTorque2 = ComputerGravityTorque_MS7(Q);
GravityTorque2',
% GravityTorque2 = [-0.0000 -205.2469  104.0030 -175.2742   98.0997 -152.7704   98.1000]