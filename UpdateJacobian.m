function JacobDHF_Base = UpdateJacobian(Q, RobotConfig)
%UpdateJacobian - part of UpdateAllFrames
% for faster speed in verify External wrech


%% initialize size of some variables
num_Joints = RobotConfig.num_Joints;
DHF_Base = zeros(4,4,8);
DHFi_DHFprevi = zeros(4,4,7);
JacobDHF_Base = zeros(6,7);


%% DH Frames computation with respect to (w.r.t.) Robot Base frame.

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


%% Jacobian matrix of EEF_Base w.r.t. Joint positions, a 6x7 matrix with order [Angular velocity; position velocity]
for index_Joint = 1:num_Joints
    % each jacobian matrix is 
    zAxis_Base = DHF_Base(1:3, 3, index_Joint);
    JacobDHF_Base(1:3, index_Joint) = zAxis_Base;
    JacobDHF_Base(4:6, index_Joint) = cross(zAxis_Base, EEF_Base(1:3,4) - DHF_Base(1:3,4, index_Joint));
end

end

    
