%% script to verify gravity compensantor with experiment data
clear all, clc;

%% load experiment data, check with freemotion for link, first test.
% folder = 'ResultsMS56Rev1_12172015\'; has_torque_data = 0;

% Does not fit the data collected in CollisionSensing preminary test, but
% satisfy with robot real time data in the section below "test with robot
% by single configuration" 
folder = 'Free_motion_forearm_link_test1\'; has_torque_data = 1;

Q_raw = importdata([folder, 'CommandedJoints.txt']);
Pose_raw = importdata([folder, 'CommandedPoses.txt']);
if (has_torque_data)
    GravityT_raw = importdata([folder, 'CommandedEffort.txt']);
end
% take a small portion for test

index_start = 10000;
index_length = 100;
% Differences between command and feedback 
time_cpp = Pose_raw(index_start:index_start+index_length-1, 2)/1000.0; % in seconds
QCommand_cpp = Q_raw(index_start:index_start+index_length-1, :) * pi/180; % in radians
Position_cpp = Pose_raw(index_start:index_start+index_length-1, 3:5)/1000 ;  % in meters
EulerXYZ_cpp = Pose_raw(index_start:index_start+index_length-1, 6:8)* pi/180 ;  % in radians

if (has_torque_data)
    GravityT_cpp = GravityT_raw(index_start:index_start+index_length-1, :) ;  % in newton meter
end

%% check with end effector pose between C++ code and matlab code;

RobotConfig = Set7DOFsParameters();
% for i_sample = 1:length(Q_experiment)
for i_sample = 1:length(QCommand_cpp)
    Q = QCommand_cpp(i_sample,:)';
    RobotFrames = UpdateAllFrames(Q,RobotConfig);
    EEF_Trans = RobotFrames.EEF_Base;
    Position_matlab(i_sample, :) = EEF_Trans(1:3,4)';
    
    PositionNorm_Error(i_sample, :)  = norm(Position_cpp(i_sample, :)) - norm(Position_matlab(i_sample, :)) ; 
end

% show errors in mm
disp(['Maximum error of position norm is: ', num2str(max(PositionNorm_Error)*1000), ' mm']);


%% check with computed joint gravity torque by Matlab code
% this folder has no torque recorded.
if(~has_torque_data)
    return;
end

% for i_sample = 1:length(Q_experiment)
for i_sample = 1:length(QCommand_cpp)
    Q = QCommand_cpp(i_sample,:)';
    GravityT_matlab(i_sample, :) = ComputerGravityTorque(Q, RobotConfig);
    
    GravityT_Error(i_sample, :)  = norm(GravityT_cpp(i_sample, :)) - norm(GravityT_matlab(i_sample, :)) ; 
end

% show errors in mm
disp(['Maximum error of gravity torque norm is: ', num2str(max(GravityT_Error)), ' newton meter']);


%% test with robot by single configuration
Rz = @(tz)[cos(tz), -sin(tz), 0; sin(tz), cos(tz), 0; 0, 0, 1];
Ry = @(ty)[cos(ty), 0, sin(ty); 0, 1, 0; -sin(ty), 0, cos(ty)];
Rx = @(tx)[ 1, 0, 0; 0, cos(tx), -sin(tx); 0, sin(tx), cos(tx)];

% Q = [180 180 180 180 180 180 180]/180*pi; % Pose1
Q = [180 90 180 180 180 180 180]/180*pi; % Pose2      // temp test to verify gravity model

% Q = [179.95709 180.22076 180.22090 179.86920 180.18906 180.01328 179.92484]/180*pi; % A Real robot configuration close to Pose1
% Q = [138.38727 219.67690 118.79504 114.11444 217.43028 215.32700 205.68263]/180*pi; % Radom Pose2
% Certain random Poses (maybe close to Singularity?) needs high precision
% (eg: decimal precision scale of 5) to reproduce the Euler Angles in Thor. 
% Q = [179.63710 132.68370 160.03061 86.18202 162.92809 234.33295 205.13373]/180*pi; % random Pose1

RobotConfig = Set7DOFsParameters();

    RobotConfig.EndEffectorMass = 5.0;
	RobotConfig.EndEffectorCOMPosition(1) = 1.0;
	RobotConfig.EndEffectorCOMPosition(2) = 2.0;
	RobotConfig.EndEffectorCOMPosition(3) = -3.0;

RobotFrames = UpdateAllFrames(Q, RobotConfig); 
EEF_Trans = RobotFrames.EEF_Base; 
Pose = [EEF_Trans(1:3,4)', mat2EulerXYZ(EEF_Trans(1:3, 1:3))*180/pi];
gravity = ComputerGravityTorque(Q, RobotConfig)'


%% Manually verify the gravity torque with this special joint configuration on Actuator 2
Q = [180 90 180 180 180 180 180]/180*pi; % Pose2      // temp test to verify gravity model
Robot = RobotConfig;

g = 9.81;
% 	Robot.LinkMass(2) = 1.34420;
% 	Robot.LinkMass(3) = 1.34420;
% 	Robot.LinkMass(4) = 1.09452;
% 	Robot.LinkMass(5) = 0.59020;
% 	Robot.LinkMass(6) = 0.59020;
% 	Robot.LinkMass(7) = 0.23948;
M2 = Robot.LinkMass(2);
M3 = Robot.LinkMass(3);
M4 = Robot.LinkMass(4);
M5 = Robot.LinkMass(5);
M6 = Robot.LinkMass(6);
M7 = Robot.LinkMass(7);
M_EE = 5;
Offset_EE = [1, 2, -3];

% 	Robot.LinkCOMPosition(1, 1) = -0.00000394;
% 	Robot.LinkCOMPosition(1, 2) = 0.001804699;
% 	Robot.LinkCOMPosition(1, 3) = -0.0748562;
% 	Robot.LinkCOMPosition(2, 1) = 0.000022339;
% 	Robot.LinkCOMPosition(2, 2) = 0.07515658;
% 	Robot.LinkCOMPosition(2, 3) = -0.0301928;
% 	Robot.LinkCOMPosition(3, 1) = -0.00002234;
% 	Robot.LinkCOMPosition(3, 2) = 0.009582299;
% 	Robot.LinkCOMPosition(3, 3) = -0.0966774;
% 	Robot.LinkCOMPosition(4, 1) = -0.00003269;
% 	Robot.LinkCOMPosition(4, 2) = 0.09546485;
% 	Robot.LinkCOMPosition(4, 3) = -0.0287837;
% 	Robot.LinkCOMPosition(5, 1) = 0.000064423;
% 	Robot.LinkCOMPosition(5, 2) = -0.00987523;
% 	Robot.LinkCOMPosition(5, 3) = -0.06073370;
% 	Robot.LinkCOMPosition(6, 1) = 0.000056819;
% 	Robot.LinkCOMPosition(6, 2) = 0.04279498;
% 	Robot.LinkCOMPosition(6, 3) = -0.00982194;
% 	Robot.LinkCOMPosition(7, 1) = 0.00000000;
% 	Robot.LinkCOMPosition(7, 2) = 0.00002000;
% 	Robot.LinkCOMPosition(7, 3) = -0.01702000;

% sign (i.e. +/-) depends if BMF_y/z align with RBF_x axis
L2 = Robot.LinkCOMPosition(2, 2); 
L3 = - Robot.LinkCOMPosition(3, 3); 
L4 = Robot.LinkCOMPosition(4, 2); 
L5 = - Robot.LinkCOMPosition(5, 3); 
L6 = Robot.LinkCOMPosition(6, 2); 
L7 = - Robot.LinkCOMPosition(7, 3); 
L_EE = - Offset_EE(3); 

A3 = 0.345/2;
A4 = 0.345;
A5 = 0.345*2 - 0.10375;
A6 = 0.345*2;
A7 = 0.345*2 + 0.10375;

% from Body2 to Actuator 2
body2_gravitytorque2 = L2*M2*g;
% from Body3 to Actuator 2
body3_gravitytorque2 = (A3 + L3)*M3*g;
% from Body4 to Actuator 2
body4_gravitytorque2 = (A4 + L4)*M4*g;
% from Body5 to Actuator 2
body5_gravitytorque2 = (A5 + L5 )*M5*g;
% from Body6 to Actuator 2
body6_gravitytorque2 = (A6 + L6)*M6*g;
% from Body7 to Actuator 2
body7_gravitytorque2 = (A7 + L7)*M7*g;
% from BodyEE to Actuator 2
bodyEE_gravitytorque2 = (A7 + L_EE)*M_EE*g;

gravitytorque2 = body2_gravitytorque2 + body3_gravitytorque2 + body4_gravitytorque2 + ... 
    body5_gravitytorque2 + body6_gravitytorque2 + body7_gravitytorque2 + bodyEE_gravitytorque2; 

gravitytorque2,
[body2_gravitytorque2, body3_gravitytorque2, body4_gravitytorque2 + ... 
    body5_gravitytorque2, body6_gravitytorque2, body7_gravitytorque2, bodyEE_gravitytorque2 ];


%% Manually verify the gravity torque with this special joint configuration on Actuator 3

% 	Robot.LinkCOMPosition(1, 1) = -0.00000394;
% 	Robot.LinkCOMPosition(1, 2) = 0.001804699;
% 	Robot.LinkCOMPosition(1, 3) = -0.0748562;
% 	Robot.LinkCOMPosition(2, 1) = 0.000022339;
% 	Robot.LinkCOMPosition(2, 2) = 0.07515658;
% 	Robot.LinkCOMPosition(2, 3) = -0.0301928;
% 	Robot.LinkCOMPosition(3, 1) = -0.00002234;
% 	Robot.LinkCOMPosition(3, 2) = 0.009582299;
% 	Robot.LinkCOMPosition(3, 3) = -0.0966774;
% 	Robot.LinkCOMPosition(4, 1) = -0.00003269;
% 	Robot.LinkCOMPosition(4, 2) = 0.09546485;
% 	Robot.LinkCOMPosition(4, 3) = -0.0287837;
% 	Robot.LinkCOMPosition(5, 1) = 0.000064423;
% 	Robot.LinkCOMPosition(5, 2) = -0.00987523;
% 	Robot.LinkCOMPosition(5, 3) = -0.06073370;
% 	Robot.LinkCOMPosition(6, 1) = 0.000056819;
% 	Robot.LinkCOMPosition(6, 2) = 0.04279498;
% 	Robot.LinkCOMPosition(6, 3) = -0.00982194;
% 	Robot.LinkCOMPosition(7, 1) = 0.00000000;
% 	Robot.LinkCOMPosition(7, 2) = 0.00002000;
% 	Robot.LinkCOMPosition(7, 3) = -0.01702000;

% sign (i.e. +/-) depends if BMF_y/z align with RBF_y axis
L3 = Robot.LinkCOMPosition(3, 2); 
L4 = - Robot.LinkCOMPosition(4, 3); 
L5 = Robot.LinkCOMPosition(5, 2); 
L6 = - Robot.LinkCOMPosition(6, 3); 
L7 = Robot.LinkCOMPosition(7, 2); 
L_EE = Offset_EE(2); 

% offset along RBF_y
A4 = 0.04; 
A5 = A4 + 0.04;
A6 = A5 + 0.0;
A7 = A6 + 0.0;


% from Body3 to Actuator 3
body3_gravitytorque3 = L3*M3*g;
% from Body4 to Actuator 3
body4_gravitytorque3 = (A4 + L4)*M4*g;
% from Body5 to Actuator 3
body5_gravitytorque3 = (A5 + L5)*M5*g;
% from Body6 to Actuator 3
body6_gravitytorque3 = (A6 + L6)*M6*g;
% from Body7 to Actuator 3
body7_gravitytorque3 = (A7 + L7)*M7*g;
% from BodyEE to Actuator 3
bodyEE_gravitytorque3 = (A7 + L_EE)*M_EE*g;

gravitytorque3 = body3_gravitytorque3 + body4_gravitytorque3 + body5_gravitytorque3 + body6_gravitytorque3 + body7_gravitytorque3 + bodyEE_gravitytorque3; 

% 104.0030
gravitytorque3,



%% Manually verify the gravity torque with this special joint configuration on Actuator 4

% 	Robot.LinkCOMPosition(1, 1) = -0.00000394;
% 	Robot.LinkCOMPosition(1, 2) = 0.001804699;
% 	Robot.LinkCOMPosition(1, 3) = -0.0748562;
% 	Robot.LinkCOMPosition(2, 1) = 0.000022339;
% 	Robot.LinkCOMPosition(2, 2) = 0.07515658;
% 	Robot.LinkCOMPosition(2, 3) = -0.0301928;
% 	Robot.LinkCOMPosition(3, 1) = -0.00002234;
% 	Robot.LinkCOMPosition(3, 2) = 0.009582299;
% 	Robot.LinkCOMPosition(3, 3) = -0.0966774;
% 	Robot.LinkCOMPosition(4, 1) = -0.00003269;
% 	Robot.LinkCOMPosition(4, 2) = 0.09546485;
% 	Robot.LinkCOMPosition(4, 3) = -0.0287837;
% 	Robot.LinkCOMPosition(5, 1) = 0.000064423;
% 	Robot.LinkCOMPosition(5, 2) = -0.00987523;
% 	Robot.LinkCOMPosition(5, 3) = -0.06073370;
% 	Robot.LinkCOMPosition(6, 1) = 0.000056819;
% 	Robot.LinkCOMPosition(6, 2) = 0.04279498;
% 	Robot.LinkCOMPosition(6, 3) = -0.00982194;
% 	Robot.LinkCOMPosition(7, 1) = 0.00000000;
% 	Robot.LinkCOMPosition(7, 2) = 0.00002000;
% 	Robot.LinkCOMPosition(7, 3) = -0.01702000;

% sign (i.e. +/-) depends if BMF_y/z align with RBF_x axis
L4 = Robot.LinkCOMPosition(4, 2); 
L5 = - Robot.LinkCOMPosition(5, 3); 
L6 = Robot.LinkCOMPosition(6, 2); 
L7 = - Robot.LinkCOMPosition(7, 3); 
L_EE = - Offset_EE(3); 

A4 = 0.345;
A5 = 0.345*2 - 0.10375;
A6 = 0.345*2;
A7 = 0.345*2 + 0.10375;

% from Body4 to Actuator 4
body4_gravitytorque4 = (L4)*M4*g;
% from Body5 to Actuator 4
body5_gravitytorque4 = (A5 - A4 + L5 )*M5*g;
% from Body6 to Actuator 4
body6_gravitytorque4 = (A6 - A4 + L6)*M6*g;
% from Body7 to Actuator 4
body7_gravitytorque4 = (A7 - A4 + L7)*M7*g;
% from BodyEE to Actuator 4
bodyEE_gravitytorque4 = (A7 - A4 + L_EE)*M_EE*g;

gravitytorque4 = body4_gravitytorque4 + body5_gravitytorque4 + body6_gravitytorque4 + body7_gravitytorque4 + bodyEE_gravitytorque4; 

gravitytorque4,
[body4_gravitytorque4 + body5_gravitytorque4, body6_gravitytorque4, body7_gravitytorque4, bodyEE_gravitytorque4];


%% Manually verify the gravity torque with this special joint configuration on Actuator 3

% 	Robot.LinkCOMPosition(7, 1) = 0.00000000;
% 	Robot.LinkCOMPosition(7, 2) = 0.00002000;
% 	Robot.LinkCOMPosition(7, 3) = -0.01702000;

% sign (i.e. +/-) depends if BMF_y/z align with RBF_y axis
L7 = Robot.LinkCOMPosition(7, 2); 
L_EE = Offset_EE(2); 

% from Body7 to Actuator 7
body7_gravitytorque7 = L7*M7*g;
% from BodyEE to Actuator 7
bodyEE_gravitytorque7 = L_EE*M_EE*g;

gravitytorque7 = body7_gravitytorque7 + bodyEE_gravitytorque7; 

gravitytorque7,

