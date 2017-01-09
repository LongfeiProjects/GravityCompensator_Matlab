function RobotJointFrames = UpdateJointFrames_MS7(Q, Tkhat_cal)
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
Ti_khat = zeros(4,4,7);

Ti_khat(:,:,1) =   [ 1   0   0   0   
              0  -1   0   0   
              0   0  -1   L0z
              0   0   0   1];
            
Ti_khat(:,:,2) =   [ 1   0   0   0   
              0   0   1   -L1y   
              0  -1   0   -L1z
              0   0   0   1];
            
Ti_khat(:,:,3) =   [ 1   0   0   0   
              0   0  -1   L2y   
              0   1   0   -L2z
              0   0   0   1];
            
Ti_khat(:,:,4) =   [ Ti_khat(1:3, 1:3, 2) [0; -L3y; -L3z];   0 0 0 1];
            
Ti_khat(:,:,5) =   [ Ti_khat(1:3, 1:3, 3) [0;  L4y; -L4z];   0 0 0 1];
            
Ti_khat(:,:,6) =   [ Ti_khat(1:3, 1:3, 2) [0;    0; -L5z];   0 0 0 1];
            
Ti_khat(:,:,7) =   [ Ti_khat(1:3, 1:3, 3) [0;  L6y;    0];   0 0 0 1];
            
T7_tool =    [ Ti_khat(1:3, 1:3, 1) [0;    0;    0];   0 0 0 1];
            
Ttool_IDM = [ 0   0   1   0   
              1   0   0   -Ltooly   
              0   1   0   Ltoolz
              0   0   0   1];
          

%% Calibration matrix (pseudo)
% for k = 1:7
%     Tkhat_cal(:,:,k) = eye(4);
% end


%% inter-joint transformation matrix T_{\hat{i},i}      
Tkhat_k = zeros(4,4,7);
for k=1:6
    Tkhat_k(:,:,k) = [-cos(Q(k)) sin(Q(k)) 0 0; -sin(Q(k)) -cos(Q(k)) 0 0; 0 0 1 0; 0 0 0 1];
end
Tkhat_k(:,:,7) = [cos(Q(7)) -sin(Q(7)) 0 0; sin(Q(7)) cos(Q(7)) 0 0; 0 0 1 0; 0 0 0 1];


%% Computer joint frames
for k=1:7
    % computer transformation matrix of frame k w.r.t. frame (k-1)
    Ti_k(:,:,k) = Ti_khat(:,:,k) * Tkhat_cal(:,:,k) * Tkhat_k(:,:,k);
    
    % computer transformation matrix of frame k w.r.t. frame 0
    if k==1
        T0_k(:,:,k) = Ti_k(:,:,k);
    else
        T0_k(:,:,k) = T0_k(:,:,k-1) * Ti_k(:,:,k);
    end
end

T0_tool = T0_k(:,:,7) * T7_tool;
T0_IDM = T0_tool * Ttool_IDM;


%% save frames to structure
RobotJointFrames.Ti_k = Ti_k;
RobotJointFrames.T0_k = T0_k;
RobotJointFrames.T0_tool = T0_tool;
RobotJointFrames.T0_IDM = T0_IDM;


end

