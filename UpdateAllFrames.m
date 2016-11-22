function RobotFrames = UpdateAllFrames(Q)
%UpdateAllFrames - Update all frames with given joint value Q.
%
%Code References:
%kinova_kinematics\Src\SerialChainModel.cpp; 
%
% Other m-files required: 
%    Set7DOFsParameters.m
%
% Inputs:
%    Q - Robot joint values, a 7x1 vector for Auris robot
%
% Outputs:
%    RobotFrames - A struct with all robot frames: DH frames, 
%
% Author: Longfei Zhao
% Created: 18-Nov-2016 


%% initialize size of some variables
if( ~exist('RobotConfig','var') )
    RobotConfig = Set7DOFsParameters();
%     disp('RobotConfig set.');
end
num_Joints = RobotConfig.num_Joints;
% n joints will have n+1 links since each joint connects two links.
num_Links = RobotConfig.num_Joints+1;

% Some important frames: Matrix of D-H Frame (prev means index starts from 0), Link Frame(start from 1), Actuator Frame(start from 1) and Link Mass Frame(start from 1).
RobotFrames.DHFprev_Base = repmat(eye(4), 1, 1, num_Joints);
RobotFrames.AF_Base = repmat(eye(4), 1, 1, num_Joints);
RobotFrames.LF_Base = repmat(eye(4), 1, 1, num_Joints);
RobotFrames.BMF_Base = repmat(eye(4), 1, 1, num_Joints);


%% Classic D-H convention and frames
% Joint1 to Joint7 refers to physical Actuator1 to Actuator7. According to
% Classic D-H convention, Frame Oi-1 is defined align with rotation axis of
% Jointi(Ji). To avoid confusion with other frames, Frame Oi is named as DHFi.
%
% Essential DH Frames are: DHF0 to DHF7, with extral Base Frame and EE Frame.
% DHF0 is located at the Actuator1 and DHF7 is a fixed frame w.r.t. DHF6 and
% located at the Actuator7(close to flange). EE Frame(EEF) is for other added
% tools, such as IDM or gripper. Robot Base Frame(BF) is at the very bottom of
% the robot base.
%
% Links are virtual concept connets DH Frames. It is not necessary to
% present the physical robot links. The fixed base is Link0, the 
% link between frame DHFi-1(Oi-1) and DHFi(Oi) is named Linki or Li.
% Ji(Actuator base) is fixed with Li-1, while Ji rotates(Actuator Rotor),
% it drives Li. Li's base connects with previous link, while tip connects
% with next link. DHFi is located at the tip of Li, and DHFi-1 is at the base
% of Li. 


%% Auris frames definition
% Joint frame (Oi) is exactly the same as DH Frames. It is essentiall a
% link frame concept. Since Oi actually is located around Ji-1, it may
% cause confusing to call it Joint Frame i. Therefore, it is rename it as
% DH Frame (DHFi).  
%
% Body Frame (BFi) is a link frame, rather than frames for physical robot
% body parts. It is renamed as Link Frame(LFi).
%
% Body COM frame (BCFi) is physically located at the center of
% Actuatori(Ji). It is renamed as Actuator Frame(AFi).
%
% COM frame (CFi) is physically located at the mass center of a physical
% link part. To avoid confusion with links, it renamed as BodyMass
% Frame(BMFi). It is consistent with "bodyMass" in the code.


%% DH frames computation

% Frame DH0 with respect to (w.r.t.) Robot Base frame.
DHF0_Base = DH2Frame(RobotConfig.Base_DH_Alpha, RobotConfig.Base_DH_Theta, RobotConfig.Base_DH_D, RobotConfig.Base_DH_R);

% In Classic D-H convention, Oi is defined align with Ji+1(i+1th Actuator).
for index_DH = 1:num_Joints
   % Transformation of DHFi w.r.t. DHFi-1(DHFprevi):
   % T^(i-1)_i = TransZi-1(di)*RotZi-1(thetai)*TransXi(ri)*RotXi(alphai);
   % Note that TransZi-1() is same as TransZi() function, and all DH
   % parameters are with same index i.
   DHFi_DHFprevi(:,:,index_DH) =  DH2Frame(RobotConfig.DH_Alpha(index_DH), Q(index_DH), RobotConfig.DH_D(index_DH), RobotConfig.DH_R(index_DH));
   
   % Computer Frame DHFi w.r.t. Robot Base frame
   if(index_DH == 1)
       % DHF1 to Base
       DHF_Base(:,:,index_DH) = DHF0_Base * DHFi_DHFprevi(:,:,index_DH);
   else
       DHF_Base(:,:,index_DH) = DHF_Base(:,:,index_DH-1) * DHFi_DHFprevi(:,:,index_DH);
   end
  
end
EEF_DHF7 = DH2Frame(RobotConfig.EndEffector_DH_Alpha, RobotConfig.EndEffector_DH_Theta, RobotConfig.EndEffector_DH_D, RobotConfig.EndEffector_DH_R);
EEF_Base = DHF_Base(:, :, 7) * EEF_DHF7;

for index_DH = 1:num_Joints
    % starting from DH0, which is the same as Joint Frame.
    if(index_DH == 1)
        DHFprev_Base(:,:,index_DH) = DHF0_Base;
    else
        DHFprev_Base(:,:,index_DH) = DHF_Base(:,:,index_DH-1) ;
    end
end

% store to RobotFrames struct
RobotFrames.DHF0_Base = DHF0_Base;
RobotFrames.DHF_Base = DHF_Base;
RobotFrames.EEF_Base = EEF_Base;
RobotFrames.DHFprev_Base = DHFprev_Base;


%% Link Frame computation
% LFi located at DHFi-1, with additional rotation of theta_i along z axis
% of DHFi-1. 

for index_Link = 1:num_Joints
    
    if(index_Link ==1)
        % Link 1 is virtual link between J1 and J2, LF1 located around J1.
        LF_Base(:,:, index_Link) = DHF0_Base * DH2Frame(0.0, Q(index_Link), 0.0, 0.0);
    else
        % LF_Base(i) = RotZ(Qi)*DHF_Base(i-1)
        LF_Base(:,:, index_Link) = DHF_Base(:,:,index_Link-1) * DH2Frame(0.0, Q(index_Link), 0.0, 0.0);
    end
    
end

RobotFrames.LF_Base = LF_Base;
% Since link EE is like Link 8, a fixed link. Its link frame is equal to
% EEF_Base. There's no need to make another one.

%% Compute Actuator Frame w.r.t. RobotBase frame
% AFi is located at the center of Actuator i. It is LFi with offset along
% its z axis to reach the physical actuator.
%
% orignial description: bodyToBodyCom:	Transformation between frame "body"
% and frame "bodyCOM" is simply a translation along the axis of the previous joint (z axis of body frame). 
for index_Actuator = 1:num_Joints
    AF_LF(:,:, index_Actuator) = DH2Frame(0.0, 0.0, RobotConfig.LinkFrameOffsetFromJoint(index_Actuator), 0.0);
    AF_Base(:,:, index_Actuator) = LF_Base(:,:, index_Actuator) * AF_LF(:,:, index_Actuator);
end

RobotFrames.AF_Base = AF_Base;


%% Compute Body Mass Frame w.r.t. Actuator Frame and Base Frame
% BMFi is a translation w.r.t. to AFi.
%
% original description: bodyCOMToCOM:	Position of the COM, in frame bodyCOM Read in KeParameters (XML file)
for index_bodyMass = 1:num_Joints-1
    % Link Mass Frame w.r.t. Actuator Frame
    BMF_AF(:,:, index_bodyMass) = [eye(3), RobotConfig.LinkCOMPosition(index_bodyMass,:)'; 0, 0, 0, 1];
end

% for the last body part (part above anctuator 7), it will consider both
% bodyMass7 (flange after Actuator 7) and end effector (not on robot by
% default). This part is renamed as bodyMass7E (7+E)
bodyMass7E = RobotConfig.LinkMass(7) + RobotConfig.EndEffectorMass;
% component of offset from bodyMass7E Frame w.r.t. Actuator 7 Frame
BMF_AF(:,:, 7) = [eye(3), (RobotConfig.LinkCOMPosition(7,:)' * RobotConfig.LinkMass(7) + EEF_DHF7(1:3,4) * RobotConfig.EndEffectorMass) / bodyMass7E; 0, 0, 0, 1];

% Compute Body Mass Frame w.r.t. Robot Base Frame
for index_bodyMass = 1:num_Joints
    BMF_Base(:,:, index_bodyMass) = AF_Base(:,:, index_bodyMass) * BMF_AF(:,:, index_bodyMass);
end

RobotFrames.BMF_Base = BMF_Base;


end

    
