
% read rtcontrol log file
clearvars
close all
clc

fprintf('Reading log files about joint torque sensor and computed joint Cartesian wrench ...\n')

%% Read raw data from data log as needed

% Ask user for the file to read
[FileName,PathNameIni,FilterIndex] = uigetfile({'*.raw;*.bin;*.log','Datalog Files (*.raw, *.bin)'},'Select a datalog', ...
                        ['C:\Users\lzhao\Documents\DATALOG-MS7\GravityCompensator\logFile\IDM_ResetSensorEachTest']);
if FileName == 0
    disp('No file Selected')
    return
end


% put generated log files in folder Output
mkdir([PathNameIni,'Output\']);
outputLog = [PathNameIni,'Output\'];
arg = [ [PathNameIni,'VVLoggerConfig.xml'], ' ', [PathNameIni, FileName], ' ', outputLog, ' ', 'TM-04-18-20']; %configXML, log file, folder output, mode
system([PathNameIni,'splitLog.exe ', arg]);

% TorqueJointCmd_nm = 'TorqueJointCmd_nm';
% TorqueJoint_nm = 'TorqueJoint_nm';
% 
% BaseCartesianWrench = 'BaseCartesianWrench';
% JointCartesianWrenchFx_n= 'JointCartesianWrenchFx_n';
% JointCartesianWrenchFy_n = 'JointCartesianWrenchFy_n';
% JointCartesianWrenchFz_n = 'JointCartesianWrenchFz_n';
% JointCartesianWrenchTx_nm = 'JointCartesianWrenchTx_nm';
% JointCartesianWrenchTy_nm = 'JointCartesianWrenchTy_nm';
% JointCartesianWrenchTz_nm = 'JointCartesianWrenchTz_nm';
% 
% RawTorque_nm = 'RawTorque_nm';


% fId         = fopen([outputLog, 'FrameIdIn.slg'],'r');
% FrameIDIn   = fread(fId,inf,'uint32');%uint32 pour actionneur = uint16
% fclose(fId);
% 
% fId           = fopen([outputLog, 'FrameIdOut.slg'],'r');
% FrameIDOut    = fread(fId,inf,'uint16'); %uint8
% fclose(fId);
% 
% fId     = fopen([outputLog, 'PositionJointCmd_deg.slg'],'r');
% command = fread(fId,inf,'float32');
% fclose(fId);
% 
% fId      = fopen([outputLog,  'PositionJoint_deg.slg'],'r');
% feedback = fread(fId,inf,'float32');
% fclose(fId);


fId         = fopen([outputLog, 'Temperature_0.slg'],'r');
Temperature_0   = fread(fId,inf,'float32');%uint32 pour actionneur = uint16
fclose(fId);

fId         = fopen([outputLog, 'DesiredJointsPos_0.slg'],'r');
DesiredJointsPos_0   = fread(fId,inf,'float64');%uint32 pour actionneur = uint16
fclose(fId);

fId           = fopen([outputLog, 'ExternalPosition_0.slg'],'r');
ExternalPosition_0    = fread(fId,inf,'float64'); %uint8
fclose(fId);

fId     = fopen([outputLog, 'RawTorque_0.slg'],'r');
RawTorque_0 = fread(fId,inf,'float64');
fclose(fId);

fId      = fopen([outputLog,  'JointCartesianWrenchTz_0.slg'],'r');
JointCartesianWrenchTz_0 = fread(fId,inf,'float32');
fclose(fId);

%% Data synchronization and pre-process
% Command is associated with FrameIdOut
% All the data comming from actuator associated with FrameIdOut

Framecommand  = [FrameIDOut, command];
Framefeedback = [FrameIDIn, feedback];
n = 1;
diffOld = inf;
IdFeedback = 1;

for i = 1:length(FrameIDOut)
    IdCmd = Framecommand(i,1);
    while Framefeedback(IdFeedback,1) ~= IdCmd
        
        IdFeedback = IdFeedback + 1;
        if IdFeedback > length(Framefeedback)
            IdFeedback = IdFeedback - 1;
            break
        end
    end
    
    if IdFeedback > i+100
        IdFeedback = i - 110;
    else
        commandSync(n,:)       = Framecommand(i,:);           %from RTControl
        feedbackSync(n,:)      = Framefeedback(IdFeedback,:); %from firmware
        %             DisturbanceSync(n,:)   = Disturbance(i,:);            %from RTControl
        n = n + 1;
    end
    %             if IdFeedback > 100 %pour gerer les erreurs de comms (fix pas beau.. il ne faut pas que ca arrive dans les 10 premiere ms...)
    %                 IdFeedback = IdFeedback - 100;
    %             end
    
end
command       = commandSync(:,2);
feedback      = feedbackSync(:,2);


%assign inputs for data analysis
rawCommand  = command(:,1);
rawFeedback = feedback(:,1);


minFdb = min(feedback);
maxFdb = max(feedback);


%change command and feedback between 0 and 360 deg
if (maxFdb-minFdb>180)
    R = 360;
    rawCommand = 180 + rawCommand;
    I = find(rawCommand>R);
    J = find(rawCommand<0);
    rawCommand(I) = rawCommand(I) - R;
    rawCommand(J) = rawCommand(J) + R;
    rawCommand = mod(rawCommand,360) -180;
    
    rawFeedback = 180 + rawFeedback;
    I = find(rawFeedback>R);
    J = find(rawFeedback<0);
    rawFeedback(I) = rawFeedback(I) - R;
    rawFeedback(J) = rawFeedback(J) + R;
    rawFeedback = rawFeedback -180;
end


%% OUTPUT AND DISPLAY 

commandedJointsPlot = rawCommand(:,1) - rawCommand(1,1);
feedbackJointsPlot  = rawFeedback(:,1) - rawFeedback(1,1);

% Figure 1
% Command and feedback
fIDfull = figure();
hold on
plot(rawCommand,'b');
plot(rawFeedback,'r');
hold off
xlabel(' sample ');
ylabel('degree');
legend('Command','Feedback')
