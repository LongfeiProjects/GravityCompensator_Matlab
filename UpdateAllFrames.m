function RobotFrames = UpdateAllFrames(Q, RobotConfig)
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
%    RobotConfig - All robot parameters, obtained from Set7DOFsParameters.m
%
% Outputs:
%    RobotFrames - A struct with all robot frames: DH frames, 
%
% Author: Longfei Zhao
% Created: 18-Nov-2016 


%% initialize size of some variables
num_Joints = RobotConfig.num_Joints;
% n joints will have n+1 links since each joint connects two links.
num_Links = RobotConfig.num_Joints+1;

% frames to hand over as output: Matrix of D-H Frame (prev means index starts from 0), Link Frame(start from 1), Actuator Frame(start from 1) and Link Mass Frame(start from 1).
RobotFrames.DHF_Base = repmat(eye(4), 1, 1, num_Joints+1);
RobotFrames.AF_Base = repmat(eye(4), 1, 1, num_Joints);
RobotFrames.LF_Base = repmat(eye(4), 1, 1, num_Links);
RobotFrames.BMF_Base = repmat(eye(4), 1, 1, num_Joints);
RobotFrames.JacobDHF_Base = repmat(eye(4), 1, 1, num_Joints);
RobotFrames.EEF_Base = eye(4);


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

%% declare size of matrix than dynamically growing its size for large
% data
AF_Base = zeros(4,4,7);
AF_LF = zeros(4,4,7);
BMF_AF = zeros(4,4,7);
BMF_Base = zeros(4,4,7);
DHF_Base = zeros(4,4,8);
DHFi_DHFprevi = zeros(4,4,7);
LF_Base  = zeros(4,4,8);
JacobDHF_Base = zeros(6,7);


%% DH Frames computation with respect to (w.r.t.) Robot Base frame.

% DH Frames start from index 0, Matlab index start from index 1. To
% explicately desplay the DH index, the following function is used to
% generate matlab matrix index. DHF_Base() is always recommended to use
% index_matrix() for the access of index.
% index_matrix = @(index_DH)index_DH+1; % However, anonymous function takes time with large size data!

index_DH = 0; % DHF0 w.r.t. Robot Base Frame
% DHF_Base(:,:,index_matrix(index_DH)) = DH2Frame(RobotConfig.Base_DH_Alpha, RobotConfig.Base_DH_Theta, RobotConfig.Base_DH_D, RobotConfig.Base_DH_R);
DHF_Base(:,:,index_DH+1) = DH2Frame(RobotConfig.Base_DH_Alpha, RobotConfig.Base_DH_Theta, RobotConfig.Base_DH_D, RobotConfig.Base_DH_R);  % However, anonymous function takes time with large size data!

% In Classic D-H convention, Oi is defined align with Ji+1(i+1th Actuator).
for index_DH = 1:num_Joints
    
   % Transformation of DHF(index_DH) w.r.t. DHF(index_DH-1)(i.e. DHFprevi).
   % T^(i-1)_i = TransZi-1(di)*RotZi-1(thetai)*TransXi(ri)*RotXi(alphai);
   % Note that TransZi-1() is same as TransZi() function, and all DH
   % parameters are with same index i.
   % e.g. the first time DH2Frame() will generate transformation matrix of
   % DHF(1) w.r.t. DHF(0). Due to the index difference, DHF(1) will be
   % corresponding to Matlab matrix DHF_Base(index_matrix(1)), i.e. DHF_Base(2)
   DHFi_DHFprevi(:,:,index_DH) =  DH2Frame(RobotConfig.DH_Alpha(index_DH), Q(index_DH), RobotConfig.DH_D(index_DH), RobotConfig.DH_R(index_DH));
   
   % Computer Frame DHFi w.r.t. Robot Base frame
   %   DHF_Base(:,:, index_matrix(index_DH)) = DHF_Base(:,:, index_matrix(index_DH-1))  * DHFi_DHFprevi(:,:,index_DH);  % However, anonymous function takes time with large size data!
   DHF_Base(:,:, index_DH+1) = DHF_Base(:,:, index_DH)  * DHFi_DHFprevi(:,:,index_DH);
  
end
EEF_DHF7 = DH2Frame(RobotConfig.EndEffector_DH_Alpha, RobotConfig.EndEffector_DH_Theta, RobotConfig.EndEffector_DH_D, RobotConfig.EndEffector_DH_R);
% EEF_Base = DHF_Base(:, :, index_matrix(7)) * EEF_DHF7;% However, anonymous function takes time with large size data!
EEF_Base = DHF_Base(:, :, 8) * EEF_DHF7;

% store to RobotFrames struct
RobotFrames.DHF_Base = DHF_Base;
RobotFrames.EEF_Base = EEF_Base;


%% Jacobian matrix of EEF_Base w.r.t. Joint positions, a 6x7 matrix with order [Angular velocity; position velocity]
for index_Joint = 1:num_Joints
    % each jacobian matrix is 
    zAxis_Base = DHF_Base(1:3, 3, index_Joint);
    JacobDHF_Base(1:3, index_Joint) = zAxis_Base;
    JacobDHF_Base(4:6, index_Joint) = cross(zAxis_Base, EEF_Base(1:3,4) - DHF_Base(1:3,4, index_Joint));
end
RobotFrames.JacobDHF_Base = JacobDHF_Base;

%% Link Frame computation
% LFi located at DHFi-1, with additional rotation of theta_i along z axis
% of DHFi-1. 

for index_Link = 1:num_Links
    
    if(index_Link < 8 )
        % LF_Base(i) = RotZ(Qi)*DHF_Base(i-1)
        % LF_Base(:,:, index_Link) = DHF_Base(:,:, index_matrix(index_Link-1))  * DH2Frame(0.0, Q(index_Link), 0.0, 0.0); % However, anonymous function takes time with large size data!
        LF_Base(:,:, index_Link) = DHF_Base(:,:, index_Link)  * DH2Frame(0.0, Q(index_Link), 0.0, 0.0);
    else
        % Link 8 is associated with fixed virtual link DHF7, therefore, no rotation
%         LF_Base(:,:, index_Link) = DHF_Base(:,:, index_matrix(index_Link-1)) * DH2Frame(0.0, 0.0, 0.0, 0.0); % However, anonymous function takes time with large size data!
        LF_Base(:,:, index_Link) = DHF_Base(:,:, index_Link) * DH2Frame(0.0, 0.0, 0.0, 0.0);
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
BMF_AF(:,:, 7) = [eye(3), (RobotConfig.LinkCOMPosition(7,:)' * RobotConfig.LinkMass(7) + RobotConfig.EndEffectorCOMPosition' * RobotConfig.EndEffectorMass) / bodyMass7E; 0, 0, 0, 1];

% Compute Body Mass Frame w.r.t. Robot Base Frame
for index_bodyMass = 1:num_Joints
    BMF_Base(:,:, index_bodyMass) = AF_Base(:,:, index_bodyMass) * BMF_AF(:,:, index_bodyMass);
end

RobotFrames.BMF_Base = BMF_Base;


end

    
