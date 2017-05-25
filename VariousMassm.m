
%% Read raw data from test: 
% load = {noload, 2kg, 3kg, 4kg}; pose  = {POSE1, POSEX2, POSEX3 POSEY1}; torqueData(iJoint, :) = [sensorJntTorque, gravityJntTorque, gravityfreeJntTorque];

% no load
load{1}.pose{1}.torqueData = [[ -6.2710	,  -5.9557	,  -0.3154	  ]
                            [ 19.3308	,  19.1634	,  0.1674	  ]
                            [ -1.6175	,  -1.9790	,  0.3615	  ]
                            [ 6.0016	,  6.1130	,  -0.1114	  ]
                            [ -0.0745	,  0.0003	,  -0.0747	  ]
                            [ 0.5557	,  0.5315	,  0.0242	  ]
                            [ -0.0312	,  -0.0000	,  -0.0312	  ]];

load{1}.pose{2}.torqueData = [[ 19.8240	,  19.1635	,  0.6605	  ]
                            [ -3.7953	,  -0.0000	,  -3.7953	  ]
                            [ 0.0405	,  -0.0001	,  0.0406	  ]
                            [ -0.1364	,  -0.0000	,  -0.1364	  ]
                            [ -0.0133	,  -0.0000	,  -0.0133	  ]
                            [ -0.0499	,  -0.0000	,  -0.0499	  ]
                            [ 0.0155	,  0.0000	,  0.0155	  ]];

load{1}.pose{3}.torqueData = [[ 6.1857	,  6.1136	,  0.0721	  ]
                            [ -1.0755	,  0.0000	,  -1.0755	  ]
                            [ 6.2052	,  6.1133	,  0.0920	  ]
                            [ 0.1026	,  -0.0000	,  0.1026	  ]
                            [ -0.0224	,  -0.0000	,  -0.0224	  ]
                            [ -0.0047	,  -0.0000	,  -0.0047	  ]
                            [ 0.0049	,  0.0000	,  0.0049	  ]];

load{1}.pose{4}.torqueData = [[ 0.5572	,  0.5314	,  0.0258	  ]
                            [ -0.1029	,  0.0000	,  -0.1029	  ]
                            [ 0.5429	,  0.5311	,  0.0118	  ]
                            [ 0.0111	,  0.0000	,  0.0111	  ]
                            [ 0.5527	,  0.5311	,  0.0215	  ]
                            [ 0.1498	,  0.0000	,  0.1498	  ]
                            [ -0.0047	,  -0.0000	,  -0.0047	  ]];
 % 2kg
load{2}.pose{1}.torqueData = [[ -9.4651	,  -9.0949	,  -0.3703	  ]
                            [ 37.3824	,  37.1628	,  0.2196	  ]
                            [ -2.7635	,  -3.5486	,  0.7851	  ]
                            [ 17.2022	,  17.3435	,  -0.1413	  ]
                            [ -0.1617	,  0.0003	,  -0.1620	  ]
                            [ 4.9706	,  4.9931	,  -0.0224	  ]
                            [ -0.0484	,  -0.0000	,  -0.0484	  ]];

load{2}.pose{2}.torqueData = [[ 37.9823	,  37.1629	,  0.8195	  ]
                            [ -7.6916	,  0.0000	,  -7.6916	  ]
                            [ 0.0793	,  -0.0001	,  0.0793	  ]
                            [ -0.4796	,  0.0000	,  -0.4796	  ]
                            [ -0.0005	,  -0.0000	,  -0.0005	  ]
                            [ -0.1701	,  0.0000	,  -0.1701	  ]
                            [ 0.0013	,  -0.0000	,  0.0013	  ]];

load{2}.pose{3}.torqueData = [[ 17.4699	,  17.3441	,  0.1258	  ]
                            [ -3.1822	,  0.0000	,  -3.1822	  ]
                            [ 17.4188	,  17.3438	,  0.0751	  ]
                            [ 0.0959	,  -0.0000	,  0.0959	  ]
                            [ -0.0277	,  -0.0000	,  -0.0276	  ]
                            [ 0.0060	,  -0.0000	,  0.0060	  ]
                            [ -0.0093	,  0.0000	,  -0.0093	  ]];

load{2}.pose{4}.torqueData = [[ 5.0243	,  4.9930	,  0.0313	  ]
                            [ -0.8235	,  0.0000	,  -0.8235	  ]
                            [ 5.0429	,  4.9927	,  0.0503	  ]
                            [ 0.0436	,  0.0000	,  0.0436	  ]
                            [ 4.9754	,  4.9927	,  -0.0173	  ]
                            [ -0.0154	,  -0.0000	,  -0.0154	  ]
                            [ 0.0041	,  0.0000	,  0.0041	  ]];
 
 % 3kg
load{3}.pose{1}.torqueData = [[ -11.1035	,  -10.6645	,  -0.4390	  ]
                            [ 45.7547	,  45.9577	,  -0.2030	  ]
                            [ -3.3954	,  -4.3334	,  0.9380	  ]
                            [ 22.6326	,  22.7539	,  -0.1213	  ]
                            [ -0.1427	,  0.0003	,  -0.1430	  ]
                            [ 7.0416	,  7.0191	,  0.0226	  ]
                            [ -0.0682	,  -0.0000	,  -0.0682	  ]];

load{3}.pose{2}.torqueData = [[ 46.8443	,  45.9577	,  0.8865	  ]
                            [ -9.1544	,  0.0000	,  -9.1544	  ]
                            [ 0.1066	,  -0.0000	,  0.1067	  ]
                            [ -0.6317	,  0.0000	,  -0.6317	  ]
                            [ -0.0235	,  -0.0000	,  -0.0235	  ]
                            [ -0.2142	,  0.0000	,  -0.2142	  ]
                            [ -0.0303	,  -0.0000	,  -0.0303	  ]];

load{3}.pose{3}.torqueData = [[ 22.9660	,  22.7545	,  0.2115	  ]
                            [ -4.3807	,  0.0000	,  -4.3808	  ]
                            [ 22.9483	,  22.7542	,  0.1941	  ]
                            [ 0.1603	,  -0.0000	,  0.1603	  ]
                            [ 0.0049	,  -0.0000	,  0.0050	  ]
                            [ -0.0168	,  -0.0000	,  -0.0168	  ]
                            [ 0.0126	,  0.0000	,  0.0126	  ]];

load{3}.pose{4}.torqueData = [[ 7.0883	,  7.0190	,  0.0693	  ]
                            [ -1.1636	,  0.0000	,  -1.1636	  ]
                            [ 7.0895	,  7.0186	,  0.0709	  ]
                            [ 0.0801	,  0.0000	,  0.0801	  ]
                            [ 7.0225	,  7.0187	,  0.0039	  ]
                            [ -0.0372	,  -0.0000	,  -0.0372	  ]
                            [ -0.0278	,  0.0000	,  -0.0278	  ]];


 % 4kg
load{4}.pose{1}.torqueData = [[ -12.6777	,  -12.2341	,  -0.4437	  ]
                            [ 54.5127	,  54.7455	,  -0.2328	  ]
                            [ -4.0073	,  -5.1182	,  1.1109	  ]
                            [ 28.0790	,  28.1572	,  -0.0782	  ]
                            [ -0.2613	,  0.0003	,  -0.2616	  ]
                            [ 9.0106	,  9.0379	,  -0.0274	  ]
                            [ -0.1169	,  -0.0000	,  -0.1169	  ]];

load{4}.pose{2}.torqueData = [[ 56.0223	,  54.7456	,  1.2767	  ]
                            [ -11.9404	,  0.0000	,  -11.9404	  ]
                            [ 0.1310	,  0.0001	,  0.1310	  ]
                            [ -0.8544	,  0.0000	,  -0.8544	  ]
                            [ -0.0274	,  -0.0000	,  -0.0274	  ]
                            [ -0.3337	,  0.0000	,  -0.3337	  ]
                            [ -0.0239	,  -0.0000	,  -0.0239	  ]];

load{4}.pose{3}.torqueData = [[ 28.4841	,  28.1579	,  0.3262	  ]
                            [ -5.3101	,  0.0000	,  -5.3102	  ]
                            [ 28.4259	,  28.1575	,  0.2684	  ]
                            [ 0.1067	,  -0.0000	,  0.1067	  ]
                            [ -0.0100	,  -0.0000	,  -0.0100	  ]
                            [ -0.0312	,  -0.0000	,  -0.0311	  ]
                            [ 0.0029	,  0.0000	,  0.0029	  ]];

load{4}.pose{4}.torqueData = [[ 9.1313	,  9.0379	,  0.0934	  ]
                            [ -1.5243	,  0.0000	,  -1.5243	  ]
                            [ 9.1362	,  9.0375	,  0.0987	  ]
                            [ 0.1303	,  0.0000	,  0.1302	  ]
                            [ 9.0526	,  9.0376	,  0.0150	  ]
                            [ -0.0558	,  0.0000	,  -0.0558	  ]
                            [ 0.0306	,  -0.0000	,  0.0306	  ]];

                        
%% plot sensor torque and gravity torque for each joint
jointSensorTorques = [];
jointGravityTorques = [];
jointGravityFreeTorques = [];
for iload=1:4
    for jpose = 1:4
        jointSensorTorques = [jointSensorTorques, load{iload}.pose{jpose}.torqueData(:, 1)];
        jointGravityTorques = [jointGravityTorques, load{iload}.pose{jpose}.torqueData(:, 2)];
        jointGravityFreeTorques = [jointGravityFreeTorques, load{iload}.pose{jpose}.torqueData(:, 3)];
    end
end

figure;
ystrings = {'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7'};
for i = 1:7
    subplot(7,1,i);
    hold on,
    plot(jointSensorTorques(i,:), 'b');
    plot(jointGravityTorques(i,:), 'g');
%     plot(jointGravityFreeTorques(i,:), 'r');
    hold off,
    
    if(i<=4) % large motor
        ylim([-60, 60]);
    else % small motor
        ylim([-12, 12]);
    end
    
    if (i==1)
        title('Sensor torque(blue) and Gravity torque(green) in Nm');
    end
    ylabel(ystrings{i});
end
    

%% find maximum gravity compensated torque for different load
max_GravityFreeTorque_load = zeros(7,4);
for iload=1:4
    for jpose = 1:4
        for kjoint = 1:7
            % update maximum torque
            temp_torque = load{iload}.pose{jpose}.torqueData(kjoint, 3);
            if abs(temp_torque) > max_GravityFreeTorque_load(kjoint, iload)
                max_GravityFreeTorque_load(kjoint, iload) = abs(temp_torque);
            end
        end
    end
end

% display result
loadString = {'NoLoad', 'Load2kg', 'Load3kg', 'Load4kg'};
jntString = {'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7' };
Table_load = array2table(max_GravityFreeTorque_load, 'VariableNames', loadString, 'RowNames', jntString),

%% find maximum gravity compensated torque for different pose
max_GravityFreeTorque_pose = zeros(7,4);
for iload=1:4
    for jpose = 1:4
        for kjoint = 1:7
            temp_torque = load{iload}.pose{jpose}.torqueData(kjoint, 3);
            % update maximum torque
            if abs(temp_torque) > max_GravityFreeTorque_pose(kjoint, jpose)
                max_GravityFreeTorque_pose(kjoint, jpose) = abs(temp_torque);
            end
        end
    end
end

% display result
poseString = {'POSE1_J246', 'POSEX2_J1', 'POSEX3_J3', 'POSEY_J5'};
jntString = {'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7' };
Table_pose = array2table(max_GravityFreeTorque_pose, 'VariableNames', poseString, 'RowNames', jntString),
 

 