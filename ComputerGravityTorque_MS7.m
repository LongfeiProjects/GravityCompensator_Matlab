function GravityTorque2 = ComputerGravityTorque_MS7(Q)
%computerGravityTorque - Compute the joint torques generated by robot
%gravity with given joint value Q, using Joint Frames in MS7. 
%
%Code References:
%computerGravityTorque.m; 
%
% Other m-files required: 
%    UpdateJointFrames_MS7.m
%
% Inputs:
%    Q - Robot joint values, a 7x1 vector for Auris robot
%
% Outputs:
%    GravityTorque2 - A 7x1 vector with gravity torque on each actuator in
%    the current configuration (Q);
% 
% Notes: 
%    For Actuator i = [1,3,5], the sign of LinkCOMPosition(i,1:2) is
%    reverted due to different definition of JointFrames in MS7.
%
% Author: Longfei Zhao
% Created: 04-Jan-2017 

%% set up mass/gravity related parameters

    num_Joints = 7;
    
    Robot.GravityAcceleration(1) = 0.0;
	Robot.GravityAcceleration(2) = 0.0;
	Robot.GravityAcceleration(3) = -9.81;
    
% values are modified according to kinova_kinematics\Src\RobotModelConfigs.cpp
	Robot.LinkMass(1) = 1.29080;
	Robot.LinkMass(2) = 1.34420;
	Robot.LinkMass(3) = 1.34420;
	Robot.LinkMass(4) = 1.09452;
	Robot.LinkMass(5) = 0.59020;
	Robot.LinkMass(6) = 0.59020;
	Robot.LinkMass(7) = 0.23948;

	Robot.LinkCOMPosition(1, 1) = -0.00000394;
	Robot.LinkCOMPosition(1, 2) = 0.001804699;
	Robot.LinkCOMPosition(1, 3) = -0.0748562;
	Robot.LinkCOMPosition(2, 1) = 0.000022339;
	Robot.LinkCOMPosition(2, 2) = 0.07515658;
	Robot.LinkCOMPosition(2, 3) = -0.0301928;
	Robot.LinkCOMPosition(3, 1) = -0.00002234;
	Robot.LinkCOMPosition(3, 2) = 0.009582299;
	Robot.LinkCOMPosition(3, 3) = -0.0966774;
	Robot.LinkCOMPosition(4, 1) = -0.00003269;
	Robot.LinkCOMPosition(4, 2) = 0.09546485;
	Robot.LinkCOMPosition(4, 3) = -0.0287837;
	Robot.LinkCOMPosition(5, 1) = 0.000064423;
	Robot.LinkCOMPosition(5, 2) = -0.00987523;
	Robot.LinkCOMPosition(5, 3) = -0.06073370;
	Robot.LinkCOMPosition(6, 1) = 0.000056819;
	Robot.LinkCOMPosition(6, 2) = 0.04279498;
	Robot.LinkCOMPosition(6, 3) = -0.00982194;
	Robot.LinkCOMPosition(7, 1) = 0.00000000;
	Robot.LinkCOMPosition(7, 2) = 0.00002000;
	Robot.LinkCOMPosition(7, 3) = -0.01702000;
    
    Robot.ToolMass = 0.0;
	Robot.ToolCOMPosition(1) = 0.0;
	Robot.ToolCOMPosition(2) = 0.0;
	Robot.ToolCOMPosition(3) = 0.0;
    
    Robot.IDMMass = 0.0;
	Robot.IDMCOMPosition(1) = 0.0;
	Robot.IDMCOMPosition(2) = 0.0;
	Robot.IDMCOMPosition(3) = 0.0;
    
% modify due to the change of JointFrame (AF)
	Robot.LinkCOMPosition(1, 1) = -Robot.LinkCOMPosition(1, 1);
	Robot.LinkCOMPosition(1, 2) = -Robot.LinkCOMPosition(1, 2);
    Robot.LinkCOMPosition(3, 1) = -Robot.LinkCOMPosition(3, 1);
	Robot.LinkCOMPosition(3, 2) = -Robot.LinkCOMPosition(3, 2);
    Robot.LinkCOMPosition(5, 1) = -Robot.LinkCOMPosition(5, 1);
	Robot.LinkCOMPosition(5, 2) = -Robot.LinkCOMPosition(5, 2);
    
RobotConfig = Robot;

    
%% test and verify Gravity compensator with ValidateFramesGravityTorques
    % remap loat of 5kg [1.0; -2.0; 3.0] w.r.t. ToolFrame, to IDMFrame
    RobotConfig.IDMMass = 5.0;
    RobotConfig.IDMCOMPosition(1) = -2.0 + 77.15/1000; 
	RobotConfig.IDMCOMPosition(2) = 3.0 - 100/1000;
	RobotConfig.IDMCOMPosition(3) = 1.0;
    warndlg('The IDM Mass and Offset are changed for verification in ComputerGravityTorque_MS7.m', 'Default parameter changed');
    % GravityTorque2 = [-0.0000 -205.2469  104.0030 -175.2742   98.0997 -152.7704   98.1000]

%% obtain robot joint frames
RobotJointFrames = UpdateJointFrames_MS7(Q);


%% Compute Body Mass Frame w.r.t. Actuator Frame and Base Frame
% BMFi is a translation w.r.t. to AFi.
%
% original description: bodyCOMToCOM:	Position of the COM, in frame bodyCOM Read in KeParameters (XML file)
for index_bodyMass = 1:num_Joints
    % Link Mass Frame w.r.t. Actuator Frame
    BMF_AF(:,:, index_bodyMass) = [eye(3), RobotConfig.LinkCOMPosition(index_bodyMass,:)'; 0, 0, 0, 1];
end

% Compute Body Mass Frame w.r.t. Robot Base Frame
for index_bodyMass = 1:num_Joints
    BMF_Base(:,:, index_bodyMass) = RobotJointFrames.T0_i(:,:, index_bodyMass) * BMF_AF(:,:, index_bodyMass);
end

ToolCOM_Base = RobotJointFrames.T0_tool*[eye(3), RobotConfig.ToolCOMPosition'; 0, 0, 0, 1];
IDMCOM_Base = RobotJointFrames.T0_IDM*[eye(3), RobotConfig.IDMCOMPosition'; 0, 0, 0, 1];


%% Compute gravity torques on each Actuator
GravityTorque2 = zeros(num_Joints,1);

% compute gravity effects on each actuator
for iActuator= 1:num_Joints
    % each outloop compute/accumulate torque for single actuator
    % same result when  reference frame is AF or DHF
    iReference_Base = RobotJointFrames.T0_i(:,:, iActuator); % take AF as reference to computer torque

    iReference_Base_z = iReference_Base(1:3, 3); % z axis is the joint axis of ith Joint

    % accumulate gravity torque effect of each(jth) body to ith Actuator.
    for jPostBody = iActuator : num_Joints
        jBMF_Base = BMF_Base(:,:,jPostBody);
        jLevelArm_Base = jBMF_Base(1:3,4) - iReference_Base(1:3,4);
        jGravityTorque_Base = cross(jLevelArm_Base, RobotConfig.LinkMass(jPostBody)*RobotConfig.GravityAcceleration');
        % project gravity torque vector to the ith Actuator(Joint) axis
%         jGravityTorque_Reference_z = dot(jGravityTorque_Base, iReference_Base_z)/norm(iReference_Base_z);
        % explicitly write dot operation to speed up for large size data
        jGravityTorque_Reference_z = sum(jGravityTorque_Base.*iReference_Base_z)/norm(iReference_Base_z);
        GravityTorque2(iActuator) = GravityTorque2(iActuator) + jGravityTorque_Reference_z;
        
%         fprintf('Actuator %d from Body mass %d, jGravityTorque_Bas = [%2.4f, %2.4f, %2.4f] \n', iActuator, jPostBody, jGravityTorque_Base(1), jGravityTorque_Base(2), jGravityTorque_Base(3));
%         fprintf('Actuator %d from Body mass %d, jLevelArm_Base = [%2.4f, %2.4f, %2.4f] \n', iActuator, jPostBody, jLevelArm_Base(1), jLevelArm_Base(2), jLevelArm_Base(3));
    end
    
    % gravity impact of tool part
    jLevelArm_Base = ToolCOM_Base(1:3,4) - iReference_Base(1:3,4);
    jGravityTorque_Base = cross(jLevelArm_Base, RobotConfig.ToolMass*RobotConfig.GravityAcceleration');
    jGravityTorque_Reference_z = sum(jGravityTorque_Base.*iReference_Base_z)/norm(iReference_Base_z);
    GravityTorque2(iActuator) = GravityTorque2(iActuator) + jGravityTorque_Reference_z;
    
    % gravity impact of IDM
    jLevelArm_Base = IDMCOM_Base(1:3,4) - iReference_Base(1:3,4);
    jGravityTorque_Base = cross(jLevelArm_Base, RobotConfig.IDMMass*RobotConfig.GravityAcceleration');
    jGravityTorque_Reference_z = sum(jGravityTorque_Base.*iReference_Base_z)/norm(iReference_Base_z);
    GravityTorque2(iActuator) = GravityTorque2(iActuator) + jGravityTorque_Reference_z;
    
   
end

end

