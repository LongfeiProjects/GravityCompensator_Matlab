

Q = rand(7,1);
% Q = ones(7,1)*pi;

RobotConfig = Set7DOFsParameters();
RobotFrames = UpdateAllFrames(Q,RobotConfig);

RobotJointFrames = UpdateJointFramesAuris7DOF(Q);

i = 2; eval(['[RobotJointFrames.T0_', num2str(i),'  zeros(4,2)      RobotFrames.AF_Base(:,:,', num2str(i),')]']);
