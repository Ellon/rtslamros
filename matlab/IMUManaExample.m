import gtsam.*;
disp('Example of application of ISAM2 for visual-inertial localization using RTSLAM data')

%% Define options for the example
options.datapath = '/home/mellon/workspace/2012-10-17_caylus-rtslam';
options.logroot = '/home/mellon/devel/var/rtslam_to_g2o/log5';
options.similarity_threshold = 0.3;

%% Read data
% Configuration files
disp('-- Reading configuration files')
setupconfig = ReadConfigFile([options.datapath '/setup.cfg']);
estimationconfig = ReadConfigFile([options.datapath '/estimation.cfg']);

% IMU data
disp('-- Reading MTI sensor data from file')
IMU_data = LoadMTIRTSLAMLog([options.datapath '/MTI.log']);

% Image timestamps
disp('-- Reading image timestamps from RTSLAM dataset')
CAMERA_data = LoadImageTimestamps(options.datapath);
firstImage = 1;

% RT-SLAM data
% TODO


%% Initialization
currentPoseGlobal = Pose3(Rot3, Point3); % initial pose is at origin. CHECK: Is it ok?
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1)); % CHECK: Is it ok?
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]); % CHECK: what does this precision mean?
sigma_init_v = noiseModel.Isotropic.Sigma(3, str2double(setupconfig.UNCERT_VLIN)); % CHECK: GTSAM example had a sigma of 1000.0, while RT-SLAM has a value of 0.
sigma_init_b = noiseModel.Isotropic.Sigmas([ str2double(setupconfig.UNCERT_ABIAS) * ones(3,1); str2double(setupconfig.UNCERT_WBIAS) * ones(3,1) ]);
sigma_between_b = [ str2double(setupconfig.UNCERT_ABIAS) * ones(3,1); str2double(setupconfig.UNCERT_WBIAS) * ones(3,1) ]; % CHECK: what's sigma_between_b?!
g = [0;0;-9.8];
w_coriolis = [0;0;0];


% %% Main Loop
% IMUtimes = [IMU_data.Time];
% 
% % Loop through the images
% for measurementIndex = firstImage:length(CAMERA_data)
%     
%     % At each non=IMU measurement we initialize a new node in the graph
%     currentPoseKey = symbol('x',measurementIndex);
%     currentVelKey =  symbol('v',measurementIndex);
%     currentBiasKey = symbol('b',measurementIndex);
%     t = CAMERA_data(measurementIndex).Time;
%     
%     if measurementIndex == firstImage
%         %% Create initial estimate and prior on initial pose, velocity, and biases
%         newValues.insert(currentPoseKey, currentPoseGlobal);
%         newValues.insert(currentVelKey, currentVelocityGlobal);
%         newValues.insert(currentBiasKey, currentBias);
%         newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
%         newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
%         newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
%     else
%         t_previous = CAMERA_data(measurementIndex-1).Time;          
%         % Integrate IMU measurements until the time of the current image
%         IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
% 
%         currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
%           currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
%           IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
% 
%         for imuIndex = IMUindices
%           accMeas = [ IMU_data(imuIndex).accelX; IMU_data(imuIndex).accelY; IMU_data(imuIndex).accelZ ];
%           omegaMeas = [ IMU_data(imuIndex).omegaX; IMU_data(imuIndex).omegaY; IMU_data(imuIndex).omegaZ ];
%           deltaT = IMU_data(imuIndex).dt;
%           currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
%         end
% 
%         % Create IMU factor
%         newFactors.add(ImuFactor( ...
%           currentPoseKey-1, currentVelKey-1, ...
%           currentPoseKey, currentVelKey, ...
%           currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
%      end
% end

%% Old code. May be re-used later.

% current_keyframe = RTSLAM_data(1);
% plot(current_keyframe.r.pose_mean(1),current_keyframe.r.pose_mean(2),'x');
% for data = RTSLAM_data(1:end)
%     if(IsKeyframe([current_keyframe.lmks.id],[data.lmks.id],options))
%         hold on
%         plot(data.r.pose_mean(1),data.r.pose_mean(2),'x');
%         hold off
%         current_keyframe = data;
%     end
% end
