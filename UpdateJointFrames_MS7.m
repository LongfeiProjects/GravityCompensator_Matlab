function RobotJointFrames = UpdateJointFrames_MS7(Q)
%UpdateJointFrames_MS7 - Update all joint frames with given joint value Q.
%Hereby, the joint frames are redefined as frames located at the center of
%Actuators. The program is deliberately written to fit the notation in
%the reference document.
%
%Reference document from Auris Surgical Robotics (105-000041-00):
%Design Document Kinematics Design of the Auris 7-DOF Robot Arm
%
% Other m-files required: 
%    None
%
% Inputs:
%    Q - Robot joint values, a 7x1 vector for Auris robot
%
% Outputs:
%    RobotFrames - A struct with all robot joint frames.
%
% Author: Longfei Zhao
% Created: 03-Jan-2017 

%% set up link offsets 
L0z = 92.75 /1000;

L1y = 40/1000;
L1z = 128.75/1000;

L2y = 172.5/1000;
L2z = 40/1000;

L3y = 40/1000;
L3z = 172.5/1000;

L4y = 241.25/1000;
L4z = 40/1000;

L5z = 103.75/1000;

L6y = 103.75/1000;

Ltooly = 77.15/1000;
Ltoolz = 100/1000;



%% computer inter-link tranformation T_{i,\hat{k}}
T0_1hat =   [ 1   0   0   0   
              0  -1   0   0   
              0   0  -1   L0z
              0   0   0   1];
            
T1_2hat =   [ 1   0   0   0   
              0   0   1   -L1y   
              0  -1   0   -L1z
              0   0   0   1];
            
T2_3hat =   [ 1   0   0   0   
              0   0  -1   L2y   
              0   1   0   -L2z
              0   0   0   1];
            
T3_4hat =   [ T1_2hat(1:3, 1:3) [0; -L3y; -L3z];   0 0 0 1];
            
T4_5hat =   [ T2_3hat(1:3, 1:3) [0;  L4y; -L4z];   0 0 0 1];
            
T5_6hat =   [ T1_2hat(1:3, 1:3) [0;    0; -L5z];   0 0 0 1];
            
T6_7hat =   [ T2_3hat(1:3, 1:3) [0;  L6y;    0];   0 0 0 1];
            
T7_tool =    [ T0_1hat(1:3, 1:3) [0;    0;    0];   0 0 0 1];
            
Ttool_IDM = [ 0   0   1   0   
              1   0   0   -Ltooly   
              0   1   0   Ltoolz
              0   0   0   1];
          

%% Calibration matrix (pseudo)
T1hat_cal = eye(4);
T2hat_cal = eye(4);
T3hat_cal = eye(4);
T4hat_cal = eye(4);
T5hat_cal = eye(4);
T6hat_cal = eye(4);
T7hat_cal = eye(4);


%% inter-joint transformation matrix T_{\hat{i},i}            
for i=1:6
    eval(['T', num2str(i), 'hat_', num2str(i), ' = [-cos(Q(', num2str(i), ')) sin(Q(', num2str(i), ')) 0 0; -sin(Q(', num2str(i), ')) -cos(Q(', num2str(i), ')) 0 0; 0 0 1 0; 0 0 0 1];']);
end

T7hat_7 = [cos(Q(7)) -sin(Q(7)) 0 0; sin(Q(7)) cos(Q(7)) 0 0; 0 0 1 0; 0 0 0 1];


%% Computer joint frames
for k=1:7
    % computer transformation matrix of frame k w.r.t. frame (k-1)
    eval(['T', num2str(k-1),'_', num2str(k),' = T', num2str(k-1),'_', num2str(k),'hat * T', num2str(k),'hat_cal * T', num2str(k),'hat_', num2str(k),';']); 
    % computer transformation matrix of frame k w.r.t. frame 0
    if k>1
        eval(['T0_', num2str(k),' = T0_', num2str(k-1),'* T', num2str(k-1),'_', num2str(k),';']);
    end
end

T0_tool = T0_7 * T7_tool;
T0_IDM = T0_tool * Ttool_IDM;


%% save frames to structure
for i=1:7
    eval(['RobotJointFrames.T0_i(:,:,i) = T0_', num2str(i),';']);
end
RobotJointFrames.T0_tool = T0_tool;
RobotJointFrames.T0_IDM = T0_IDM;


end

