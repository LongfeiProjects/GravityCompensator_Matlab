
% read rtcontrol log file
clearvars
close all
clc

fprintf('Reading log files about joint torque sensor and computed joint Cartesian wrench ...\n')

%% Read raw data from data log as needed
rootDir = ['C:\Users\', getenv('username'), '\Documents\DATALOG-MS7\GravityCompensator\'];
logFilesDir = [rootDir, 'logFile\IDM_ResetSensorEachTest\'];

% 15 configurations in total
for i = 1:15
    % joint wrench at each configuration
    logFilePose = [logFilesDir, 'logFile_Pose', num2str(i), '.log'];
    Output_PoseDir{i} = [logFilesDir, 'Output_Pose', num2str(i), '\'];
    mkdir(Output_PoseDir{i});
    arg = [ [rootDir,'VVLoggerConfig_JointWrench.xml '], logFilePose, ' both ', Output_PoseDir{i}, ' TM-04-18-20'];
    system( [rootDir, 'splitLog.exe ', arg] );
    
    % joint wrench at 'zero-troque' pose after each configuration. Ideally
    % torque along joint axis Tz should be zero.
    logFileZero = [logFilesDir, 'logFile_Pose0', num2str(i), '.log'];
    Output_ZeroDir{i} = [logFilesDir, 'Output_Pose0', num2str(i), '\'];
    mkdir(Output_ZeroDir{i});
    arg = [ [rootDir,'VVLoggerConfig_JointWrench.xml '], logFilePose, ' both ', Output_ZeroDir{i}, ' TM-04-18-20'];
    system( [rootDir, 'splitLog.exe ', arg] );
end


%% Extract needed values from generated txt files

for iRobotConfig = 1:15
    
    RobotConfig(iRobotConfig).BaseJointCartesianWrench = [  mean( importdata([Output_PoseDir{iRobotConfig}, 'BaseJointCartesianWrenchFx.txt']) ), ...
        mean( importdata([Output_PoseDir{iRobotConfig}, 'BaseJointCartesianWrenchFy.txt']) ), ...
        mean( importdata([Output_PoseDir{iRobotConfig}, 'BaseJointCartesianWrenchFz.txt']) ), ...
        mean( importdata([Output_PoseDir{iRobotConfig}, 'BaseJointCartesianWrenchTx.txt']) ), ...
        mean( importdata([Output_PoseDir{iRobotConfig}, 'BaseJointCartesianWrenchTy.txt']) ), ...
        mean( importdata([Output_PoseDir{iRobotConfig}, 'BaseJointCartesianWrenchTz.txt']) )];
        
    for jActuator = 1:7
        RobotConfig(iRobotConfig).Actuator(jActuator).Position = mean( importdata([Output_PoseDir{iRobotConfig}, 'ExternalPosition_', num2str(jActuator-1),'.txt']) );
        RobotConfig(iRobotConfig).Actuator(jActuator).Command = mean( importdata([Output_PoseDir{iRobotConfig}, 'DesiredJointsPos_', num2str(jActuator-1),'.txt']) );
        
        RobotConfig(iRobotConfig).Actuator(jActuator).CartesianWrench = [   mean( importdata([Output_PoseDir{iRobotConfig}, 'JointCartesianWrenchFx_', num2str(jActuator-1),'.txt']) ), ...
            mean( importdata([Output_PoseDir{iRobotConfig}, 'JointCartesianWrenchFy_', num2str(jActuator-1),'.txt']) ), ...
            mean( importdata([Output_PoseDir{iRobotConfig}, 'JointCartesianWrenchFz_', num2str(jActuator-1),'.txt']) ), ...
            mean( importdata([Output_PoseDir{iRobotConfig}, 'JointCartesianWrenchTx_', num2str(jActuator-1),'.txt']) ), ...
            mean( importdata([Output_PoseDir{iRobotConfig}, 'JointCartesianWrenchTy_', num2str(jActuator-1),'.txt']) ), ...
            mean( importdata([Output_PoseDir{iRobotConfig}, 'JointCartesianWrenchTz_', num2str(jActuator-1),'.txt']) )];
        
        RobotConfig(iRobotConfig).Actuator(jActuator).RawTorque = mean( importdata([Output_PoseDir{iRobotConfig}, 'RawTorque_', num2str(jActuator-1),'.txt']) );
        RobotConfig(iRobotConfig).Actuator(jActuator).RawTorqueVector = importdata([Output_PoseDir{iRobotConfig}, 'RawTorque_', num2str(jActuator-1),'.txt']); % very noisy
    end
    
end


%% OUTPUT AND DISPLAY 

% commandedJointsPlot = rawCommand(:,1) - rawCommand(1,1);
% feedbackJointsPlot  = rawFeedback(:,1) - rawFeedback(1,1);
% 
% % Figure 1
% % Command and feedback
% fIDfull = figure();
% hold on
% plot(rawCommand,'b');
% plot(rawFeedback,'r');
% hold off
% xlabel(' sample ');
% ylabel('degree');
% legend('Command','Feedback')
