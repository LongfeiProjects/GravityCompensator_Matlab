function Robot = Set7DOFsParameters()
%Set7DOFsParameters - Set D-H parameters for Auris robot.
%
%Transforming from functions:
% 1. RobotModelBase::Set7DOFsParameters() from
%kinova_kinematics\Src\RobotModelBase.cpp; 
% 2. AurisRobotMS5::Initialize(RobotConfig cfg) from kinova
%kinova_kinematics\Src\RobotModelConfigs.cpp; 
%
% Outputs:
%    Robot - Robot parameters and configurations
%
% Author: Longfei Zhao
% Created: 17-Nov-2016 


%% Set parameters
    ASR_PI = 3.1415926535897932384626433832795;
    RAD = @(x)x*ASR_PI/180.0;
    
    Robot.num_Joints = 7;
    
	Robot.GravityAcceleration(1) = 0.0;
	Robot.GravityAcceleration(2) = 0.0;
	Robot.GravityAcceleration(3) = -9.81;
    
    % Auris robot setup in the Cart on their site
    Robot.GravityAcceleration(1) = -9.81;
	Robot.GravityAcceleration(2) = 0.0;
	Robot.GravityAcceleration(3) = 0.0;

	Robot.DH_Alpha(1) = ASR_PI / 2.0;
	Robot.DH_Alpha(2) = ASR_PI / 2.0;
	Robot.DH_Alpha(3) = ASR_PI / 2.0;
	Robot.DH_Alpha(4) = ASR_PI / 2.0;
	Robot.DH_Alpha(5) = ASR_PI / 2.0;
	Robot.DH_Alpha(6) = ASR_PI / 2.0;
	Robot.DH_Alpha(7) = ASR_PI;

	Robot.DH_Theta(1) = 0.0;
	Robot.DH_Theta(2) = 0.0;
	Robot.DH_Theta(3) = 0.0;
	Robot.DH_Theta(4) = 0.0;
	Robot.DH_Theta(5) = 0.0;
	Robot.DH_Theta(6) = 0.0;
	Robot.DH_Theta(7) = 0.0;

	Robot.DH_D(1) = -0.12875;
	Robot.DH_D(2) = -0.08;
	Robot.DH_D(3) = -0.345;
	Robot.DH_D(4) = -0.08;
	Robot.DH_D(5) = -0.345;
	Robot.DH_D(6) = 0.0;
	Robot.DH_D(7) = -0.10375;

	Robot.DH_R(1) = 0.0;
	Robot.DH_R(2) = 0.0;
	Robot.DH_R(3) = 0.0;
	Robot.DH_R(4) = 0.0;
	Robot.DH_R(5) = 0.0;
	Robot.DH_R(6) = 0.0;
	Robot.DH_R(7) = 0.0;

    Robot.Base_DH_Alpha = ASR_PI; % Inclination of the base 3*PI/4;
	Robot.Base_DH_Theta = 0.0;
    Robot.Base_DH_R = 0.0;
	Robot.Base_DH_D = 0.09275;

	Robot.EndEffector_DH_Alpha = 0.0;
	Robot.EndEffector_DH_Theta = 0.0;
	Robot.EndEffector_DH_R = 0.0;
	Robot.EndEffector_DH_D = 0.0;

    
 	Robot.LinkFrameOffsetFromJoint(1) = 0.0;
 	Robot.LinkFrameOffsetFromJoint(2) = -0.04;
 	Robot.LinkFrameOffsetFromJoint(3) = -0.1725;
 	Robot.LinkFrameOffsetFromJoint(4) = -0.04;
 	Robot.LinkFrameOffsetFromJoint(5) = -0.24125;
 	Robot.LinkFrameOffsetFromJoint(6) = 0.0;
 	Robot.LinkFrameOffsetFromJoint(7) = -0.10375;

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
    
    Robot.EndEffectorMass = 0.0;
	Robot.EndEffectorCOMPosition(1) = 0.0;
	Robot.EndEffectorCOMPosition(2) = 0.0;
	Robot.EndEffectorCOMPosition(3) = 0.0;
    
end
