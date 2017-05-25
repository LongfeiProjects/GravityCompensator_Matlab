clc, clear all;

%% Pre-defined robot configurations: Pose(PoseIndex, ActuatorIndex)

% Group A: configuration from Auris
Pose(1 , :) = [ 115,   236.85,  138.20,  264.09,  63    ,  235.28,  155.28  ];
Pose(2 , :) = [ 140,   245.42,  90.92 ,  281.41,  353.62,  295.06,  265.40  ];
% Pose(3 , :) = [ 293,   137.05,  220.52,  87.40 ,  217.98,  307.70,  103.13  ]; % Q?? Q6 cannot reach 307.70
Pose(3 , :) = [ 293.0001,   137.0502,  220.5201,  87.3998 ,  217.9778,  307.4591,  103.1300];
Pose(4 , :) = [ 10 ,   176.73,  147.59,  43.32 ,  142.68,  275   ,  205.12  ];
Pose(5 , :) = [ 85 ,   255.93,  141.29,  236.09,  65.84 ,  228.26,  130.50  ];
Pose(6 , :) = [ 140,   205.51,  96.12 ,  275.16,  313.20,  260.62,  229.57  ];
Pose(7 , :) = [ 120,   240.31,  140.20,  239.88,  66.74 ,  239.24,  115.38  ];
Pose(8 , :) = [ 285,   113.25,  129.73,  289.67,  7.89  ,  195.54,  83.44   ];
Pose(9 , :) = [ 285,   121.22,  234.46,  131.94,  356.65,  77.01 ,  227.27  ];
Pose(10, :) = [ 340,   141.74,  22.27 ,  67.95 ,  16.12 ,  61.73 ,  153.11  ];

% Goup B: configuration for extrem load on joints
Pose(11, :) = [ 180,   180,     180,     180,     180,     180,     180     ];
Pose(12, :) = [ 270,   180,     180,     180,     180,     180,     180     ];
Pose(13, :) = [ 180,   180,     180,     180,     180,     90 ,     180     ];
Pose(14, :) = [ 180,   180,     180,     90 ,     180,     180,     180     ];
Pose(15, :) = [ 180,   90 ,     180,     180,     180,     180,     180     ];


                        
                        
%% From Offline PC: Reference data from RobotBase Class
JointWrench_PC{1} = [     0.0000	110.2625	  0.0000	-27.9005	  0.0000	-10.7114	
                         90.2280	-42.0707	 -0.2484	 -6.9044	-14.8713	 10.7851	
                         41.3099	-62.1115	-34.8631	 -7.1290	 -2.4050	 -4.1627	
                         42.5445	  1.1795	 48.6356	 -0.7967	  2.0313	  0.6476	
                        -31.7672	-34.5177	  0.8023	  0.9116	 -0.6450	  8.3476	
                          9.1143	-19.3155	 23.1839	 -3.1432	  0.7029	  1.8213	
                        -10.5665	-15.8279	-14.8006	 -2.3085	  0.0888	  1.5531	
                          2.2934	 12.3686	 10.9768	  1.1963	 -0.1851	 -0.0414 ];

                    
JointWrench_PC{2} = [       0.0000	110.2625	  0.0000	-29.0945	  0.0000	-15.2101	
                             64.0037	-76.2533	 -0.2559	-13.4362	-11.3291	 15.2878	
                             22.4690	-47.8244	-63.1487	-13.4574	 -1.3893	 -3.7361	
                             49.9289	 16.8400	 37.4215	  0.3224	 -1.9633	  0.4533	
                            -33.6530	-30.3433	 12.1667	  1.4061	  0.6292	  5.4584	
                             23.3638	 -5.7106	 20.3759	 -0.3649	  0.7207	  0.6204	
                            -21.6473	 -9.6480	 -4.4205	 -0.6654	  0.1670	  2.8940	
                              4.3091	-14.6683	  6.7086	 -1.0595	 -0.3473	 -0.0789  ];

% With pre-defined Pose3
% JointWrench_PC{3} = [   0.0000	110.2625	  0.0000	-33.0071	  0.0000	 -4.2595	
%                         -91.6344	 38.9122	 -0.2802	  8.4017	 19.8165	  4.3504	
%                         -55.4597	-51.6549	 32.1835	  8.1475	 -3.1831	  8.9312	
%                         -16.4769	 47.4563	 40.6603	  4.6365	 -3.1336	  5.5362	
%                          30.0832	-10.4019	 34.4685	  6.1231	 -0.7706	 -5.5767	
%                          30.2298	  5.7464	  6.8406	  0.0974	 -0.0860	 -0.3580	
%                         -18.2406	-15.1553	  4.3397	  0.5020	  0.1764	  2.7261	
%                           5.7429	 11.6217	 10.5208	  1.1278	 -0.4630	 -0.1042];

% In robot test, predefined Pose3 isn't achieved precisely. 
% Actual Pose3 in Nova is: Pose(3 , :) = [293.0001,   137.0502,  220.5202,  87.3999 ,  217.9800,  307.0117,  103.1300];
% disp('In robot test, predefined Pose3 is not achieved precisely. ');
% disp('Actual Pose3 in Nova is: Pose(3 , :) = [293.0001,   137.0502,  220.5202,  87.3999 ,  217.9800,  307.0117,  103.1300]');
% JointWrench_PC{3} = [     0.0000	110.2625	  0.0000	-32.9979	  0.0000	 -4.2819	
% -91.6344	 38.9122	 -0.2801	  8.3980	 19.8080	  4.3727	
% -55.4597	-51.6547	 32.1839	  8.1602	 -3.2019	  8.9227	
% -16.4765	 47.4567	 40.6601	  4.6406	 -3.1483	  5.5550	
%  30.0830	-10.4015	 34.4687	  6.1417	 -0.7657	 -5.5913	
%  30.2298	  5.7466	  6.8401	  0.1028	 -0.1090	 -0.3629	
% -18.0570	-15.3734	  4.3399	  0.5005	  0.1752	  2.7031	
%   5.7145	 11.4974	 10.6717	  1.1206	 -0.4607	 -0.1037];	

% In robot test, predefined Pose3 isn't achieved precisely. 
% Actual Pose3 in Nova is: Pose(3 , :) = [293.0001,   137.0502,  220.5201,  87.3998 ,  217.9778,  307.4591,  103.1300];
disp('In robot test, predefined Pose3 is not achieved precisely. ');
disp('Actual Pose3 in Nova is: Pose(3 , :) = [293.0001,   137.0502,  220.5201,  87.3998 ,  217.9778,  307.4591,  103.1300]');
JointWrench_PC{3} = [       0.0000	110.2625	  0.0000	-33.0040	  0.0000	 -4.2674	
                        -91.6344	 38.9123	 -0.2802	  8.4005	 19.8137	  4.3582	
                        -55.4598	-51.6547	 32.1837	  8.1520	 -3.1897	  8.9284	
                        -16.4767	 47.4566	 40.6601	  4.6381	 -3.1387	  5.5428	
                         30.0831	-10.4017	 34.4686	  6.1296	 -0.7688	 -5.5817	
                         30.2296	  5.7477	  6.8404	  0.0993	 -0.0940	 -0.3598	
                        -18.1765	-15.2318	  4.3407	  0.5016	  0.1760	  2.7181	
                          5.7336	 11.5782	 10.5737	  1.1253	 -0.4622	 -0.1041];	

                    
JointWrench_PC{4} = [     0.0000	110.2625	  0.0000	-38.3470	  0.0000	-10.5508	
                         17.2851	 98.0420	 -0.3135	 26.4410	 -4.6275	 10.6594	
                         14.3259	  1.1273	 81.0758	 15.3720	 -9.0529	 -2.5903	
                        -24.4358	 59.8232	 -0.9653	  2.9876	  1.3559	  8.4035	
                         12.4384	-12.5632	 43.4595	 10.6354	  2.6477	 -2.2785	
                        -11.0271	 28.3432	  8.2899	  2.4216	  1.8578	 -3.1305	
                         -5.6340	  8.8952	 21.6879	  3.1600	 -0.0349	  0.8352	
                         -2.7765	-15.2768	 -6.1352	 -1.3424	  0.2239	  0.0501];


JointWrench_PC{5} = [     0.0000	110.2625	  0.0000	-21.8889	  0.0000	  1.0714	
                         99.1779	  8.6484	 -0.2109	  0.9034	-10.3845	 -1.0176	
                         20.4923	-79.4216	  7.2152	  4.6768	  1.3335	  1.3956	
                          9.1725	 14.5715	 62.2928	  2.6582	  7.8753	 -2.2336	
                        -33.6567	-30.9342	 10.5629	  3.5988	 -0.7618	  9.2358	
                          2.6686	-23.5784	 20.7507	 -3.9006	  1.3210	  2.0026	
                        -10.4651	-12.0837	-18.0471	 -2.6637	 -0.0410	  1.5721	
                         -4.8581	 13.5951	  8.3847	  1.2479	  0.3911	  0.0889	];


JointWrench_PC{6} = [     0.0000	110.2625	  0.0000	-41.3390	  0.0000	 -9.8292	
                         63.9923	-76.2625	 -0.3322	-22.8353	-19.2045	  9.9477	
                         47.9955	-22.1932	-63.1164	-15.6580	 -0.8587	-11.6049	
                         53.3696	 32.1151	 17.2383	  3.8238	 -5.7290	 -1.1652	
                        -15.7614	-37.5749	 23.2589	  2.9839	  0.1206	  2.2168	
                         18.6649	 -3.0918	 25.2134	 -0.5528	  0.4130	  0.4599	
                        -16.6440	-17.2741	 -2.4115	 -0.5477	  0.2275	  2.1506	
                          8.7615	 -7.6370	 11.9852	 -0.3970	 -0.7062	 -0.1598];


JointWrench_PC{7} = [     0.0000	110.2625	  0.0000	-30.3916	  0.0000	-16.2528	
                         86.2253	-49.7617	 -0.2640	 -9.3939	-16.3640	 16.3348	
                         35.8362	-61.5774	-41.2769	-12.9911	 -3.4497	 -6.1324	
                         42.4780	 -6.9454	 48.2103	 -2.7305	 -3.2327	  1.9402	
                        -14.5759	-44.3036	 -5.0994	 -0.3795	 -0.2291	  3.0747	
                          6.9247	 -7.6567	 29.7839	 -1.2322	  1.8808	  0.7700	
                        -16.8131	-16.2460	 -5.8840	 -0.9727	  0.0918	  2.5258	
                          1.2517	 12.2517	 11.2721	  1.1923	 -0.1012	 -0.0224];


JointWrench_PC{8} = [     0.0000	110.2625	  0.0000	-41.6081	  0.0000	 -4.0651	
                        -96.1578	 25.7810	 -0.3338	  7.7879	 29.1016	  4.1846	
                        -31.4516	-73.0478	 21.3213	  9.1282	  1.2300	 17.6793	
                        -28.5746	 -8.1046	 57.3991	 -4.1670	 15.2767	  0.0826	
                        -32.4121	 33.4107	 -5.8692	 -0.7283	  1.2336	 11.0443	
                         22.0986	  0.9281	-22.4598	  0.1770	 -5.1055	 -0.0368	
                         20.8636	 12.0613	  0.6845	  0.1257	 -0.0441	 -3.0544	
                          2.1752	-14.2654	 -8.3964	 -1.3021	 -0.1750	 -0.0400];


JointWrench_PC{9} = [   0.0000	110.2625	  0.0000	-34.1627	  0.0000	 -3.2104	
                        -96.1574	 25.7831	 -0.2874	  5.8654	 21.9116	  3.3051	
                        -41.2146	-68.0213	 21.3131	  7.4623	 -1.2347	 10.4899	
                         -5.0225	 35.9412	 53.4776	  7.6988	 -3.3496	  2.9742	
                         26.6124	-28.4942	 26.0985	  5.1056	  1.0156	 -4.0973	
                        -16.9103	-18.5851	 19.0339	 -1.2814	 -0.9022	 -2.0193	
                         17.1159	 -9.3113	-14.1978	 -2.0967	 -0.0791	 -2.4758	
                         -0.8903	 15.3887	  6.4129	  1.3565	  0.0715	  0.0167];


JointWrench_PC{10} = [     0.0000	110.2625	  0.0000	-46.7740	  0.0000	 -1.2363	
                        -34.0280	 93.5578	 -0.3660	 33.1730	 12.0708	  1.3730	
                        -22.2479	-17.2923	 77.3679	 18.9244	 11.6219	  8.0395	
                         -6.9219	-62.8097	 13.5602	 -9.2615	 -1.2932	-10.7176	
                         11.0114	 -0.9614	-45.5971	 -9.8970	 -3.4205	 -2.3179	
                          1.5046	 31.4799	  0.6355	  1.4357	 -0.1297	  3.0274	
                         -0.1335	  1.3078	 24.0729	  3.3758	  0.1322	  0.0115	
                          7.6796	-14.7976	 -0.8873	 -1.2082	 -0.6186	 -0.1403];


JointWrench_PC{11} = [     0.0000	110.2625	  0.0000	-60.0957	  0.0000	  0.5790	
                         -0.0279	-99.5535	 -0.4490	-48.6286	  0.0154	 -0.3978	
                          0.0299	  0.5215	-82.3378	-36.8271	  0.2276	 -0.0119	
                         -0.0988	-64.6257	 -0.6051	-24.5641	  0.0388	 -0.1339	
                         -0.1413	  0.4880	-46.9149	-14.9518	  0.0505	  0.0456	
                         -0.1645	-31.5199	 -0.3527	 -6.2864	  0.0330	 -0.0191	
                         -0.1811	  0.2801	-24.1065	 -3.3560	  0.0099	  0.0253	
                          0.0639	 16.6940	 -0.1898	  1.3414	 -0.0051	 -0.0007];


JointWrench_PC{12} = [     0.0000	110.2625	  0.0000	-60.1380	  0.0000	 11.9388	
                        -99.5535	  0.0583	 -0.4491	  0.0816	 48.7087	-11.7573	
                        -82.3389	  0.2600	 -0.1382	 -0.0371	  7.8676	 36.8944	
                        -64.6286	 -0.0495	 -0.0258	 -0.0171	 24.6098	 -4.3307	
                        -46.9173	 -0.2017	  0.0248	 -0.0010	  2.0661	 14.9749	
                        -31.5221	  0.0631	  0.0738	  0.0118	  6.2903	 -0.3345	
                        -24.1080	 -0.1638	  0.1018	  0.0115	  0.4008	  3.3607	
                         16.6946	 -0.0082	  0.1434	  0.0020	 -1.3449	 -0.3040];


JointWrench_PC{13} = [    0.0000	110.2625	  0.0000	-56.6655	  0.0000	  3.8584	
                         -0.0366	-99.5536	 -0.4276	-45.2094	  0.0325	 -3.6885	
                          0.0609	  0.4979	-82.3379	-33.4211	  3.5325	 -0.0034	
                         -0.0477	-64.6260	 -0.5707	-21.1663	  0.0461	 -3.4498	
                         -0.0990	  0.4579	-46.9154	-11.5694	  3.3854	  0.0575	
                         -0.0987	-31.5206	 -0.3107	 -2.9090	  0.0422	 -3.3618	
                         -0.2519	 -0.1215	-24.1071	 -3.3628	  0.0111	  0.0351	
                          0.1123	 16.6946	  0.0885	  1.3465	 -0.0090	 -0.0015];


JointWrench_PC{14} = [    0.0000	110.2625	  0.0000	-44.8698	  0.0000	 15.2538	
                         -0.0669	-99.5539	 -0.3540	-33.4516	  0.0762	-15.1232	
                          0.1689	  0.4165	-82.3382	-21.7091	 15.0160	  0.0314	
                          0.1300	-64.6269	 -0.4524	 -9.4833	  0.0857	-14.9708	
                         -0.3544	  0.0480	-46.9163	-14.9718	  0.0599	  0.1131	
                         -0.3070	-31.5207	 -0.0570	 -6.2897	  0.0613	 -0.0206	
                         -0.2894	  0.0536	-24.1070	 -3.3598	  0.0117	  0.0404	
                          0.1385	 16.6946	 -0.0327	  1.3443	 -0.0112	 -0.0020];


JointWrench_PC{15} = [   0.0000	110.2625	  0.0000	-22.7448	  0.0000	 36.9146	
                         -0.1244	-99.5542	 -0.2159	-11.3989	  0.0942	-36.8576	
                         -0.2640	  0.3741	-82.3382	-36.8419	  0.2564	  0.1193	
                         -0.3289	-64.6259	 -0.4888	-24.5722	  0.1262	 -0.1500	
                         -0.3079	  0.4028	-46.9150	-14.9556	  0.0578	  0.0986	
                         -0.2762	-31.5197	 -0.2953	 -6.2870	  0.0553	 -0.0203	
                         -0.2664	  0.2359	-24.1061	 -3.3567	  0.0113	  0.0372	
                          0.1229	 16.6940	 -0.1590	  1.3420	 -0.0099	 -0.0017];



%% From Robot test: Computed joint Cartesian Wrench on each actuator: JointWrench{PoseIndex} (ActuatorIndex, WrenchIndex)
JointWrench_RobotBase{1} = [   0  110.26       0  -27.90       0  -10.71;
                           90.2279  -42.0710   -0.2484   -6.9044  -14.8713   10.7852;
                           41.3100  -62.1113  -34.8634   -7.1291   -2.4050   -4.1627;
                           42.5447    1.1794   48.6355   -0.7967    2.0313    0.6476;
                          -31.7672  -34.5177    0.8022    0.9116   -0.6450    8.3476;
                            9.1144  -19.3155   23.1839   -3.1432    0.7028    1.8213;
                          -10.5665  -15.8280  -14.8006   -2.3084    0.0888    1.5531;
                            2.2934   12.3686   10.9768    1.1963   -0.1851   -0.0414];

                        
JointWrench_RobotBase{2} = [       0  110.26       0  -29.09       0  -15.21;
                               64.0034  -76.2536   -0.2559  -13.4363  -11.3290   15.2879;
                               22.4689  -47.8241  -63.1489  -13.4575   -1.3893   -3.7361;
                               49.9291   16.8400   37.4213    0.3224   -1.9634    0.4534;
                              -33.6529  -30.3434   12.1667    1.4061    0.6292    5.4584;
                               23.3637   -5.7106   20.3760   -0.3649    0.7207    0.6204;
                              -21.6474   -9.6480   -4.4204   -0.6654    0.1670    2.8940;
                                4.3091  -14.6683    6.7086   -1.0595   -0.3473   -0.0789];
                   
% In robot test, predefined Pose3 isn't achieved precisely. 
% Actual Pose3 in Nova is: Pose(3 , :) = [293.0001,   137.0502,  220.5202,  87.3999 ,  217.9800,  307.0117,  103.1300];              
% JointWrench_RobotBase{3} = [     0.00     110.26  0.00    -33.00  0.00    -4.28;
%                                -91.6344   38.9122	-0.2801	8.3980	19.8079	4.3732;
%                                -55.4597    -51.6547	32.1838	  8.1609	-3.2030   8.9222;
%                                -16.4766   47.4566   40.6601   4.6399    -3.1456   5.5516;
%                                30.0830    -10.4016  34.4687   6.1339    -0.7678   -5.5852;
%                                30.2298    5.7466    6.8403    0.1014    -0.1029   -0.3616;
%                                -18.0570   -15.3734  4.3399    0.5005    0.1752    2.7031;
%                                5.7193     11.5186   10.6463   1.1218    -0.4611   -0.1038];
                           
% In robot test, predefined Pose3 isn't achieved precisely. 
% Actual Pose3 in Nova is: Pose(3 , :) = [293.0001,   137.0502,  220.5201,  87.3998 ,  217.9778,  307.4591,  103.1300];                          
JointWrench_RobotBase{3} = [    0.00        110.26      0.00    -33.00  0.00    -4.27;
                                -91.6344    38.9123     -0.2802 8.4005  19.8136 4.3583;
                                -55.4597    -51.6547    32.1838 8.1520  -3.1897 8.9284;
                                -16.4767    47.4566     40.6601 4.6381  -3.1387 5.5428;
                                30.0831     -10.4016    34.4686 6.1296  -0.7688 -5.5817;
                                30.2296     5.7477      6.8403  0.0993  -0.0940 -0.3598;
                                -18.1765    -15.2318    4.3407  0.5016  0.1760  2.7181;
                                5.7336  11.5781 10.5737 1.1253  -0.4622 -0.1041];     
                                
                   
JointWrench_RobotBase{4} = [     0.00     110.26  0.00    -38.35  0.00    -10.55;
                               17.2855    98.0419   -0.3135   26.4410   -4.6276   10.6595;
                               14.3262    1.1273    81.0757   15.3719   -9.0529   -2.5904;
                               -24.4355   59.8233   -0.9653   2.9876    1.3559    8.4035;
                               12.4382    -12.5630  43.4596   10.6355   2.6477    -2.2785;
                               -11.0275   28.3430   8.2898    2.4216    1.8578    -3.1305;
                               -5.6339    8.8955    21.6878   3.1599    -0.0349   0.8352;
                               -2.7766    -15.2767  -6.1354   -1.3424   0.2239    0.0501];

                   
JointWrench_RobotBase{5} = [     0.00     110.26  0.00    -21.89  0.00    1.07;
                               99.1780    8.6483    -0.2109   0.9034    -10.3844  -1.0176;
                               20.4922    -79.4216  7.2151    4.6768    1.3335    1.3956;
                               9.1725     14.5715   62.2928   2.6582    7.8753    -2.2336;
                               -33.6568   -30.9341  10.5629   3.5988    -0.7618   9.2358;
                               2.6686     -23.5785  20.7506   -3.9006   1.3210    2.0026;
                               -10.4651   -12.0837  -18.0472  -2.6637   -0.0410   1.5721;
                               -4.8582    13.5951   8.3846    1.2479    0.3911    0.0889];


JointWrench_RobotBase{6} = [    0.00    110.26  0.00    -41.34  0.00    -9.83;
                                63.9920   -76.2628  -0.3322   -22.8354  -19.2045  9.9477;
                                47.9954   -22.1929  -63.1166  -15.6581  -0.8587   -11.6049;
                                53.3698   32.1150   17.2381   3.8238    -5.7291   -1.1652;
                                -15.7612  -37.5751  23.2588   2.9838    0.1206    2.2168;
                                18.6648   -3.0918   25.2135   -0.5528   0.4131    0.4599;
                                -16.6440  -17.2741  -2.4115   -0.5477   0.2275    2.1506;
                                8.7616    -7.6370   11.9851   -0.3970   -0.7062   -0.1598];
    

JointWrench_RobotBase{7} = [     0.00     110.26  0.00    -30.39  0.00    -16.25;
                               86.2251    -49.7621  -0.2640   -9.3939   -16.3640  16.3349;
                               35.8362    -61.5771  -41.2772  -12.9912  -3.4498   -6.1325;
                               42.4782    -6.9456   48.2101   -2.7306   -3.2327   1.9402;
                               -14.5758   -44.3036  -5.0996   -0.3795   -0.2291   3.0747;
                               6.9247     -7.6566   29.7839   -1.2322   1.8808    0.7700;
                               -16.8130   -16.2460  -5.8839   -0.9726   0.0918    2.5258;
                               1.2517     12.2516   11.2722   1.1923    -0.1012   -0.0224];


JointWrench_RobotBase{8} = [     0.00     110.26  0.00    -41.61  0.00    -4.07;
                               -96.1578   25.7811   -0.3338   7.7880    29.1016   4.1846;
                               -31.4521   -73.0476  21.3213   9.1282    1.2300    17.6794;
                               -28.5749   -8.1049   57.3990   -4.1670   15.2766   0.0827;
                               -32.4118   33.4110   -5.8694   -0.7283   1.2336    11.0442;
                               22.0984    0.9283    -22.4600  0.1770    -5.1054   -0.0368;
                               20.8635    12.0615   0.6846    0.1257    -0.0440   -3.0543;
                               2.1753     -14.2653  -8.3965   -1.3021   -0.1750   -0.0400];


JointWrench_RobotBase{9} = [     0.00     110.26  0.00    -34.16  0.00    -3.21;
                               -96.1574   25.7832   -0.2874   5.8654    21.9117   3.3051;
                               -41.2149   -68.0211  21.3132   7.4623    -1.2347   10.4900;
                               -5.0226    35.9414   53.4775   7.6988    -3.3496   2.9743;
                               26.6123    -28.4941  26.0987   5.1056    1.0156    -4.0973;
                               -16.9102   -18.5852  19.0339   -1.2814   -0.9021   -2.0194;
                               17.1159    -9.3112   -14.1979  -2.0967   -0.0791   -2.4758;
                               -0.8902    15.3887   6.4129    1.3565    0.0715    0.0167];


JointWrench_RobotBase{10} = [     0.00    110.26  0.00    -46.77  0.00    -1.24;
                                -34.0280  93.5578   -0.3660   33.1730   12.0708   1.3730;
                                -22.2480  -17.2922  77.3679   18.9244   11.6219   8.0395;
                                -6.9217   -62.8098  13.5601   -9.2615   -1.2932   -10.7176;
                                11.0113   -0.9613   -45.5971  -9.8970   -3.4205   -2.3179;
                                1.5047    31.4799   0.6354    1.4357    -0.1297   3.0274;
                                -0.1337   1.3078    24.0729   3.3758    0.1322    0.0116;
                                7.6797    -14.7975  -0.8873   -1.2082   -0.6186   -0.1403];
   
                        
JointWrench_RobotBase{11} = [      0  110.26       0  -60.10       0    0.58;
                               -0.0278  -99.5535   -0.4490  -48.6286    0.0154   -0.3977;
                                0.0299    0.5215  -82.3378  -36.8271    0.2275   -0.0119;
                               -0.0988  -64.6257   -0.6051  -24.5641    0.0388   -0.1339;
                               -0.1413    0.4880  -46.9150  -14.9518    0.0505    0.0455;
                               -0.1645  -31.5199   -0.3527   -6.2864    0.0330   -0.0191;
                               -0.1811    0.2801  -24.1065   -3.3560    0.0099    0.0253;
                                0.0639   16.6940   -0.1898    1.3414   -0.0051   -0.0007] ;

                
JointWrench_RobotBase{12} = [     0.00    110.26  0.00    -60.14  0.00    11.94;
                                -99.5535  0.0580    -0.4491   0.0814    48.7087   -11.7573;
                                -82.3389  0.2609    -0.1385   -0.0371   7.8676    36.8944;
                                -64.6286  -0.0489   -0.0265   -0.0172   24.6098   -4.3307;
                                -46.9173  -0.2009   0.0246    -0.0010   2.0661    14.9749;
                                -31.5221  0.0629    0.0773    0.0118    6.2903    -0.3345;
                                -24.1080  -0.1634   0.1017    0.0115    0.4008    3.3607;
                                16.6946   -0.0081   0.1431    0.0020    -1.3449   -0.3040];

                    
JointWrench_RobotBase{13} = [     0.00    110.26  0.00    -56.67  0.00    3.86;
                                -0.0365   -99.5536  -0.4276   -45.2094  0.0324    -3.6885;
                                0.0610    0.4979    -82.3379  -33.4211  3.5325    -0.0034;
                                -0.0476   -64.6260  -0.5707   -21.1663  0.0460    -3.4498;
                                -0.0989   0.4579    -46.9154  -11.5694  3.3854    0.0574;
                                -0.0986   -31.5206  -0.3107   -2.9090   0.0422    -3.3618;
                                -0.2519   -0.1215   -24.1071  -3.3628   0.0111    0.0351;
                                0.1123    16.6946   0.0884    1.3465    -0.0090   -0.0015];


JointWrench_RobotBase{14} = [    0.00	110.26	0.00	-44.87	0.00	15.25;
                                 -0.0665  -99.5539  -0.3540   -33.4516  0.0761    -15.1231;
                                 0.1692   0.4165    -82.3382  -21.7091  15.0159   0.0314;
                                 0.1305   -64.6269  -0.4524   -9.4833   0.0857    -14.9708;
                                 -0.3544  0.0483    -46.9163  -14.9718  0.0599    0.1131;
                                 -0.3070  -31.5207  -0.0572   -6.2897   0.0613    -0.0206;
                                 -0.2894  0.0538    -24.1070  -3.3598   0.0117    0.0404;
                                 0.1385   16.6946   -0.0328   1.3443    -0.0112   -0.0020];


JointWrench_RobotBase{15} = [     0.00    110.26  0.00    -22.74  0.00    36.91;
                                -0.1233   -99.5542  -0.2159   -11.3989  0.0940    -36.8574;
                                -0.2640   0.3751    -82.3382  -36.8418  0.2564    0.1193;
                                -0.3290   -64.6259  -0.4896   -24.5722  0.1262    -0.1500;
                                -0.3079   0.4033    -46.9150  -14.9556  0.0578    0.0987;
                                -0.2762   -31.5197  -0.2956   -6.2870   0.0553    -0.0203;
                                -0.2664   0.2362    -24.1061  -3.3567   0.0113    0.0372;
                                0.1229    16.6940   -0.1592   1.3420    -0.0099   -0.0017];
                            
%% From Robot test: JointTorque --  sensor data from joint torque sensor
for i = 1:15
    JointWrench_Sensor{i} = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
end
                            
%% JointWrench_Error = JointWrench_PC - JointWrench_RobotBase
JointWrench_Error_ForceX = [];
JointWrench_Error_ForceY = [];
JointWrench_Error_ForceZ = [];
JointWrench_Error_TorqueX = [];
JointWrench_Error_TorqueY = [];
JointWrench_Error_TorqueZ = [];

for i = 1:15
    JointWrench_Error{i} = JointWrench_PC{i} - JointWrench_RobotBase{i};
    
    JointWrench_Error_ForceX = [JointWrench_Error_ForceX JointWrench_Error{i}(:,1)];
    JointWrench_Error_ForceY = [JointWrench_Error_ForceY JointWrench_Error{i}(:,2)];
    JointWrench_Error_ForceZ = [JointWrench_Error_ForceZ JointWrench_Error{i}(:,3)];
    JointWrench_Error_TorqueX = [JointWrench_Error_TorqueX JointWrench_Error{i}(:,4)];
    JointWrench_Error_TorqueY = [JointWrench_Error_TorqueY JointWrench_Error{i}(:,5)];
    JointWrench_Error_TorqueZ = [JointWrench_Error_TorqueZ JointWrench_Error{i}(:,6)];
end

h = figure;
subplot(2,3,1), imagesc(JointWrench_Error_ForceX), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('Force X Error ("offline PC" - "Robot Base")');
ylabel('Force error in for each Actuator ');
xlabel('Number of Test Sample');
subplot(2,3,2), imagesc(JointWrench_Error_ForceY), 
caxis([-5, 5]*0.001), colorbar,cellborder();
title('Force Y Error ("offline PC" - "Robot Base")');
ylabel('Force error in for each Actuator ');
xlabel('Number of Test Sample');
subplot(2,3,3), imagesc(JointWrench_Error_ForceZ), 
caxis([-5, 5]*0.001), colorbar,cellborder();
title('Force Z Error ("offline PC" - "Robot Base")');
ylabel('Force error in for each Actuator ');
xlabel('Number of Test Sample');

subplot(2,3,4), imagesc(JointWrench_Error_TorqueX), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('Torque X Error ("offline PC" - "Robot Base")');
ylabel('Torque error in for each Actuator ');
xlabel('Number of Test Sample');
subplot(2,3,5), imagesc(JointWrench_Error_TorqueY), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('Torque Y Error ("offline PC" - "Robot Base")');
ylabel('Torque error in for each Actuator ');
xlabel('Number of Test Sample');
subplot(2,3,6), imagesc(JointWrench_Error_TorqueZ), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('Torque Z Error ("offline PC" - "Robot Base")');
ylabel('Torque error in for each Actuator ');
xlabel('Number of Test Sample');
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.



%% Save figure for report (large and clear)
h1 = figure, imagesc(JointWrench_Error_ForceX), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('ForceX Error',  'FontSize', 30);
ylabel('Actuator index', 'FontSize', 30);
xlabel('Test index', 'FontSize', 30);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gca,'fontsize',30) ;
saveas(h1,'Result1.png'); close(h1);

h2 = figure, imagesc(JointWrench_Error_ForceY), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('ForceY Error',  'FontSize', 30);
ylabel('Actuator index', 'FontSize', 30);
xlabel('Test index', 'FontSize', 30);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gca,'fontsize',30) ;
saveas(h2,'Result2.png'); close(h2);

h3 = figure, imagesc(JointWrench_Error_ForceZ), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('ForceZ Error',  'FontSize', 30);
ylabel('Actuator index', 'FontSize', 30);
xlabel('Test index', 'FontSize', 30);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gca,'fontsize',30) ;
saveas(h3,'Result3.png'); close(h3);

h4 = figure, imagesc(JointWrench_Error_TorqueX), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('TorqueX Error',  'FontSize', 30);
ylabel('Actuator index', 'FontSize', 30);
xlabel('Test index', 'FontSize', 30);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gca,'fontsize',30) ;
saveas(h4,'Result4.png'); close(h4);

h5 = figure, imagesc(JointWrench_Error_TorqueY), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('TorqueY Error',  'FontSize', 30);
ylabel('Actuator index', 'FontSize', 30);
xlabel('Test index', 'FontSize', 30);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gca,'fontsize',30) ;
saveas(h5,'Result5.png'); close(h5);

h6 = figure, imagesc(JointWrench_Error_TorqueZ), 
caxis([-5, 5]*0.001), colorbar, cellborder();
title('TorqueZ Error',  'FontSize', 30);
ylabel('Actuator index', 'FontSize', 30);
xlabel('Test index', 'FontSize', 30);
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gca,'fontsize',30) ;
saveas(h6,'Result6.png'); close(h6);



%% Generate Latex file

texFolder = '../TMxx-report/';
texFileMaster = [texFolder, 'MASTER.tex'];
texFileTestReults = [texFolder, 'testResults.tex'];

copyfile('./Result.png', [texFolder, 'images/Result.png']);


texPath = [texFolder,'appendix.tex'];
texPathWithValue = texPath;
texPathWithValue(end-3:end) = []; %remove extension
texPathWithValue = [texPathWithValue,'_withvalue.tex']; %New File name

% jointIndex = 0 is the base
latexfileTemplate = fileread(texPath);
latexfileNew = '';

% use the table template for 15 tests, each test appears as a subsection in
% appendix.c
for poseIndex = 1:15

    latexfileNew = strcat(latexfileNew, latexfileTemplate);
    latexfileNew = strrep(latexfileNew,   'VAR_POSE_NUM', num2str(poseIndex) );
    latexfileNew = strrep(latexfileNew,   'VAR_POSE', ['[ ', num2str(Pose(poseIndex,:)), ']'] );
    latexfileNew = strrep(latexfileNew,   'VAR_LABEL_PC', ['wrech_PC_Pose', num2str(poseIndex)] );
    latexfileNew = strrep(latexfileNew,   'VAR_LABEL_ROBOT', ['wrech_Robot_Pose', num2str(poseIndex)] );
    latexfileNew = strrep(latexfileNew,   'VAR_LABEL_ERROR', ['wrech_Error_Pose', num2str(poseIndex)] );
    latexfileNew = strrep(latexfileNew,   'VAR_LABEL_SENSOR', ['wrech_Sensor_Pose', num2str(poseIndex)] );

    for wrenchIndex = 1:6
        for jointIndex = 0:7 
            % copy Joint wrench PC data to table
            latexfileNew = strrep(latexfileNew,   ['VAR_PC_', num2str(wrenchIndex), num2str(jointIndex)], num2str(JointWrench_PC{poseIndex}(jointIndex+1, wrenchIndex)) );
            % copy Joint wrench Robot data to table
            latexfileNew = strrep(latexfileNew,   ['VAR_ROBOT_', num2str(wrenchIndex), num2str(jointIndex)], num2str(JointWrench_RobotBase{poseIndex}(jointIndex+1, wrenchIndex)) );
            % copy Joint wrench Error data to table
            latexfileNew = strrep(latexfileNew,   ['VAR_ERROR_', num2str(wrenchIndex), num2str(jointIndex)], num2str(JointWrench_Error{poseIndex}(jointIndex+1, wrenchIndex)) );
        end
    end

    % copy Joint wrench Sensor data to table
    for jointIndex = 1:7
        latexfileNew = strrep(latexfileNew,   ['VAR_SENSOR_', num2str(jointIndex)], num2str(JointWrench_Sensor{poseIndex}(jointIndex)) );
    end

end

fileID = fopen(texPathWithValue,'w');
fprintf(fileID,'%s',latexfileNew);
fclose(fileID);

folderleft = cd(texFolder);
system(['PdfLaTex ', texFileMaster]);
% system(['PdfLaTex ', texFileMaster]);
cd(folderleft);
copyfile([texFolder, 'MASTER.pdf'],[texFolder,'GravityModelReport.pdf']);

fprintf('\nGenerating Latex report has finished without any issues.\n')





