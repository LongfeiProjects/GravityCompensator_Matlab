function GravityTorque = ComputerGravityTorque(Q, RobotConfig)
%computerGravityTorque - Compute the joint torques generated by robot
%gravity with given joint value Q. 
%
%Code References:
%kinova_kinematics\Src\SerialChainModel.cpp; 
%
% Other m-files required: 
%    Set7DOFsParameters.m
%    UpdateAllFrames.m
%
% Inputs:
%    Q - Robot joint values, a 7x1 vector for Auris robot
%    RobotConfig - All robot parameters, obtained from Set7DOFsParameters.m
%
% Outputs:
%    GravityTorque - A 7x1 vector with gravity torque on each actuator in
%    the current configuration (Q);
%
% Author: Longfei Zhao
% Created: 21-Nov-2016 


%% initialize size of some variables
num_Joints = RobotConfig.num_Joints;


%% provide joint values and update all frames
% Q = ones(7,1)*pi;
RobotFrames = UpdateAllFrames(Q, RobotConfig);

%% Compute gravity torques on each Actuator
GravityTorque = zeros(num_Joints,1);
bodyMass = zeros(num_Joints,1);
% update body mass
for iActuator= 1:num_Joints
    % for the last body part (part above anctuator 7), it will consider
    % both bodyMass7 (flange after Actuator 7) and end effector (not on
    % robot by default). This part is renamed as bodyMass7E (7+E) in BMF_AF 
    if(iActuator == 7)
        bodyMass(iActuator) = RobotConfig.LinkMass(iActuator) + RobotConfig.EndEffectorMass;
    else
        bodyMass(iActuator) = RobotConfig.LinkMass(iActuator);
    end
end

% compute gravity effects on each actuator
for iActuator= 1:num_Joints
    % each outloop compute/accumulate torque for single actuator
    % same result when  reference frame is AF or DHF
%     iReference_Base = RobotFrames.AF_Base(:,:, iActuator); % take AF as reference to computer torque

%     DHIndex2MatIndex = @(index_DH)index_DH+1; % However, anonymous function takes time with large size data!
%     iReference_Base = RobotFrames.DHF_Base(:,:, iActuator-1+1); % take DHF as reference to computer torque, same as Joint Frame in the code.
    % iActuator-1 is the index of DH. However, DH index starts from 0 in the desing. To build up matrix, matrix index = DH_index + 1 
    iReference_Base = RobotFrames.DHF_Base(:,:, iActuator-1+1); % take DHF as reference to computer torque, same as Joint Frame in the code.
    
    iReference_Base_z = iReference_Base(1:3, 3); % z axis is the joint axis of ith Joint

    % accumulate gravity torque effect of each(jth) body to ith Actuator.
    for jPostBody = iActuator : num_Joints
        jBMF_Base = RobotFrames.BMF_Base(:,:,jPostBody);
        jLevelArm_Base = jBMF_Base(1:3,4) - iReference_Base(1:3,4);
        jGravityTorque_Base = cross(jLevelArm_Base, bodyMass(jPostBody)*RobotConfig.GravityAcceleration');
        % project gravity torque vector to the ith Actuator(Joint) axis
%         jGravityTorque_Reference_z = dot(jGravityTorque_Base, iReference_Base_z)/norm(iReference_Base_z);
        % explicitly write dot operation to speed up for large size data
        jGravityTorque_Reference_z = sum(jGravityTorque_Base.*iReference_Base_z)/norm(iReference_Base_z);
        GravityTorque(iActuator) = GravityTorque(iActuator) + jGravityTorque_Reference_z;
        
%         disp(['', num2str(), '', num2str(), '', num2str()]);
%         fprintf('Actuator %d from Body mass %d, jGravityTorque_Bas = [%2.4f, %2.4f, %2.4f] \n', iActuator, jPostBody, jGravityTorque_Base(1), jGravityTorque_Base(2), jGravityTorque_Base(3));
        fprintf('Actuator %d from Body mass %d, jLevelArm_Base = [%2.4f, %2.4f, %2.4f] \n', iActuator, jPostBody, jLevelArm_Base(1), jLevelArm_Base(2), jLevelArm_Base(3));
        
    end
            
    % store it to the joint information. except gravity torque, joint
    % information should include position, velocity, raw torques, etc.
    % Joint.GravityTorque(iActuator)  = GravityTorque(iActuator);
   
end
    fprintf('bodyMass is [%2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f] \n', bodyMass(1), bodyMass(2), bodyMass(3), bodyMass(4), bodyMass(5), bodyMass(6), bodyMass(7));


end
