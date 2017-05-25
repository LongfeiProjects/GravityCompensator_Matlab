function output = extractDataFromLog()

%% read log data obtained in NOVA Auris Control panel
logfolder = 'C:\thor-logs\';
% Ask user for the file to readcmd
[FileName,PathNameIni,FilterIndex] = uigetfile({'*.csv; *.log; *.xsl'},'Select a datalog',logfolder);
if FileName == 0
    disp('No file Selected')
    return
end


%% load the data as table and matrix
rawDataTable = readtable([PathNameIni,FileName], 'Delimiter', ';', 'ReadVariableNames', true);

% regulate time stamps as running time in seconds
sizeTable = size(rawDataTable);
for i = 1:sizeTable(1)
    timeStringCell{i} = strsplit(rawDataTable.Time{i},':');
    timeStamp(i) = str2num(timeStringCell{i}{1})*3600 + str2num(timeStringCell{i}{2})*60 + str2num(timeStringCell{i}{3});
    TimePass(i,1) = timeStamp(i) - timeStamp(1);
end
rawDataTable.Time = [];
dataTable = join(table([0:sizeTable-1]', TimePass), rawDataTable, 'LeftKeys',1,'RightKeys',1);

% generate matrix
dataMatrix = table2array(dataTable);


%% index for different features in the raw matrix
indexLogCount = 1;
indexTimePass = 2;
indexBaseTemperature = 3;
for indexActuator = 1:13
    indexTemperature(indexActuator) = 4 + (indexActuator-1)*6;
    indexTorque(indexActuator)      = 5 + (indexActuator-1)*6; 
    indexCurrent(indexActuator)     = 6 + (indexActuator-1)*6; 
    indexMotorCurrent(indexActuator) = 7 + (indexActuator-1)*6; 
    indexAngle(indexActuator)       = 8 + (indexActuator-1)*6; 
    indexCommand(indexActuator)     = 9 + (indexActuator-1)*6; 
end
indexPose = [82, 86, 90, 84, 88, 92];
indexDesirePose = indexPose + 1;


%% output values needed

% return mean value of joint torque sensor data
meanTorque = mean( dataMatrix(:,indexTorque) );

% encapsulate data in the output
output.meanTorque = meanTorque;


end
