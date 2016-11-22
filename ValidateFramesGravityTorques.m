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

% for i_sample = 1:length(Q_experiment)
for i_sample = 1:length(QCommand_cpp)
    Q = QCommand_cpp(i_sample,:)';
    RobotFrames = UpdateAllFrames(Q);
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
    GravityT_matlab(i_sample, :) = ComputerGravityTorque(Q);
    
    GravityT_Error(i_sample, :)  = norm(GravityT_cpp(i_sample, :)) - norm(GravityT_matlab(i_sample, :)) ; 
end

% show errors in mm
disp(['Maximum error of gravity torque norm is: ', num2str(max(GravityT_Error)), ' newton meter']);


%% test with robot by single configuration
Rz = @(tz)[cos(tz), -sin(tz), 0; sin(tz), cos(tz), 0; 0, 0, 1];
Ry = @(ty)[cos(ty), 0, sin(ty); 0, 1, 0; -sin(ty), 0, cos(ty)];
Rx = @(tx)[ 1, 0, 0; 0, cos(tx), -sin(tx); 0, sin(tx), cos(tx)];

% Q = [180 180 180 180 180 180 180]/180*pi; % Pose1
% Q = [180 135 180 90 180 225 180]/180*pi; % Pose2
Q = [249.02 170.38 153.37 57.70 198.19 228.19 171.08]/180*pi; % random Pose

RobotFrames = UpdateAllFrames(Q); 
EEF_Trans = RobotFrames.EEF_Base; 
Pose = [EEF_Trans(1:3,4)', mat2EulerXYZ(EEF_Trans(1:3, 1:3))*180/pi],
gravity = ComputerGravityTorque(Q)'





