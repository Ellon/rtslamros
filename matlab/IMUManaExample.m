clear;
import gtsam.*;
disp('Example of application of ISAM2 for visual-inertial localization using RTSLAM data')

%% Define options for the example
options.datapath = '/home/mellon/workspace/2012-10-17_caylus-rtslam';
options.rtslamlogpath = '/home/mellon/devel/var/rtslam_to_g2o/log5';
options.rtslamlogsize = 1000;
options.similarity_threshold = 0.3;

%% Read data
% Configuration files
disp('-- Reading configuration files from RTSLAM dataset')
setupconfig = ReadConfigFile([options.datapath '/setup.cfg']);
estimationconfig = ReadConfigFile([options.datapath '/estimation.cfg']);

% IMU data
disp('-- Reading MTI sensor data from RTSLAM dataset')
IMU_data = LoadMTIRTSLAMLog([options.datapath '/MTI.log']);
% Correct IMU timestamps
for i=1:numel(IMU_data)
    IMU_data(i).Time = IMU_data(i).Time + str2double(setupconfig.IMU_TIMESTAMP_CORRECTION);
end

% Image timestamps
% disp('-- Reading image timestamps from RTSLAM dataset')
% CAMERA_data = LoadImageTimestamps(options.datapath);

% Load start time
disp('-- Reading start time from RTSLAM dataset')
fileID = fopen([options.datapath '/sdate.log']);
start_date = fscanf(fileID,'%f');
fclose(fileID);

% RT-SLAM data (robot poses and landmarks measurements)
disp('-- Reading log from RTSLAM run')
logfiles = dir(fullfile(options.rtslamlogpath,'*.log'));
fulllogfiles = cellfun(@(S) fullfile(options.rtslamlogpath, S),{logfiles(1:end).name}, 'Uniform', 0);
RTSLAM_data = arrayfun(@(x) ReadRTSLAMLog(x),fulllogfiles(1:options.rtslamlogsize));

noiseModelGPS = noiseModel.Diagonal.Precisions([ 1.0/(((2/180)*pi)^2) * [1;1;1]; 1.0/0.07 * [1;1;1] ]);
firstRTSLAMPose = 1;
RTSLAMskip = 10; % Skip this many RTSLAM data each time


%% Initialization
currentPoseGlobal = Pose3(Rot3(quat2dcm(RTSLAM_data(firstRTSLAMPose).r.pose_mean(4:7)')), ...
                          Point3(RTSLAM_data(firstRTSLAMPose).r.pose_mean(1:3))); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Sigmas([ 0; 0; 0; 0; 0; 0 ]); % We are quite sure the robot is at origin at the begining
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0); % Even if the robot is stationary, we give an uncertainty on it's velocity.
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]); % Values taken from Kitti example
sigma_between_b = [ 1.67e-4 * ones(3,1); 2.91e-6 * ones(3,1) ]; % Values taken from IMU_metadata.AccelerometerBiasSigma and IMU_metadata.GyroscopeBiasSigma, from Kitti example
g = [0;0;-9.8];
w_coriolis = [0;0;0];

camera_intrinsic = ParseUblasVector(setupconfig.CAMERA1_INTRINSIC);
camera_pose_vector = ParseUblasVector(setupconfig.CAMERA1_POSE_INERTIAL);
camera_pose_vector(4:6) = (camera_pose_vector(4:6)/180)*pi;
camera_pose = Pose3(Rot3(angle2dcm(camera_pose_vector(6),camera_pose_vector(5),camera_pose_vector(4),'ZYX')), ...
                          Point3(camera_pose_vector(1:3)'));
camera_calibration = Cal3_S2(camera_intrinsic(3), camera_intrinsic(4), 0, camera_intrinsic(1), camera_intrinsic(2));

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

%% Main Loop
% (1) we read the measurements
% (2) we create the corresponding factors in the graph
% (3) we solve the graph to obtain and optimal estimate of robot trajectory
IMUtimes = [IMU_data.Time];
trackedlmks = [];
adding_euc_lmks = false;
movie_index = 1;

disp('-- Starting main loop')
% isamPlotted = false;
% Loop through the images
for measurementIndex = firstRTSLAMPose:length(RTSLAM_data)
    
    % At each non=IMU measurement we initialize a new node in the graph
    currentPoseKey = symbol('x',measurementIndex);
    currentVelKey =  symbol('v',measurementIndex);
    currentBiasKey = symbol('b',measurementIndex);
    t = RTSLAM_data(measurementIndex).r.date;

    trackedlmks = TrackRTSLAMLmks(RTSLAM_data(measurementIndex),trackedlmks,currentPoseKey);

    
    if measurementIndex == firstRTSLAMPose
        %% Create initial estimate and prior on initial pose, velocity, and biases
        newValues.insert(currentPoseKey, currentPoseGlobal);
        newValues.insert(currentVelKey, currentVelocityGlobal);
        newValues.insert(currentBiasKey, currentBias);
        newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
        newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
        newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
    else
        t_previous = RTSLAM_data(measurementIndex-1).r.date;
        % Integrate IMU measurements until the time of the current image
        IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
                
        currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
            currentBias, ...
            0.01 .^ 2 * eye(3), ... % Value taken from IMU_metadata.AccelerometerSigma
            1.75e-4 .^ 2 * eye(3), ... % Value taken from IMU_metadata.GyroscopeSigma
            0.0 .^ 2 * eye(3)); % Value taken from IMU_metadata.IntegrationSigma
        
        for imuIndex = IMUindices
            accMeas = [ IMU_data(imuIndex).accelX; IMU_data(imuIndex).accelY; IMU_data(imuIndex).accelZ ];
            omegaMeas = [ IMU_data(imuIndex).omegaX; IMU_data(imuIndex).omegaY; IMU_data(imuIndex).omegaZ ];
            deltaT = IMU_data(imuIndex).dt;
            currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
        end
        
        % Create IMU factor
        newFactors.add(ImuFactor( ...
            currentPoseKey-1, currentVelKey-1, ...
            currentPoseKey, currentVelKey, ...
            currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
        
        % Bias evolution as given in the IMU metadata
        % CHECK: How does the bias evolute in RT-SLAM?
        newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
            noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));
        
        % Create GPS factor
        RTSLAMPose = Pose3(Rot3(quat2dcm(RTSLAM_data(measurementIndex).r.pose_mean(4:7)')), ...
                           Point3(RTSLAM_data(measurementIndex).r.pose_mean(1:3)));
        if mod(measurementIndex, RTSLAMskip) == 0 %CHECK: Add GPS measurements each GPSskip nodes
            newFactors.add(PriorFactorPose3(currentPoseKey, RTSLAMPose, noiseModelGPS));
        end
        
        % Add initial value
        newValues.insert(currentPoseKey, RTSLAMPose);
        newValues.insert(currentVelKey, currentVelocityGlobal); % CHECK: maybe add current velocity from the filter?
        newValues.insert(currentBiasKey, currentBias); % CHECK: maybe add current bias from the filter?
    
        % Check for euclidean landmarks and add factors if any.
        eucLmkIndexes = find([trackedlmks.is_euclidean] > 0);
        if numel(eucLmkIndexes) > 8
            adding_euc_lmks = true;
            for lmkIndex = eucLmkIndexes
                currentLmkKey = symbol('l',lmkIndex);
                for lmkMeasIndex = 1:numel(trackedlmks(lmkIndex).robot_pose_key)
                    newFactors.add(GenericProjectionFactorCal3_S2( Point2(trackedlmks(lmkIndex).meas(:,:,lmkMeasIndex)), ...
                        noiseModel.Isotropic.Sigma(2, 1.0), ...
                        uint64(trackedlmks(lmkIndex).robot_pose_key(lmkMeasIndex)), ...
                        currentLmkKey, ...
                        camera_calibration, ...
                        camera_pose.inverse));
                end
                trackedlmks(lmkIndex).meas = [];
                trackedlmks(lmkIndex).robot_pose_key = uint64([]);
                if ~trackedlmks(lmkIndex).being_updated
                    if ~newValues.exists(currentLmkKey)
                        newValues.insert(currentLmkKey,Point3(trackedlmks(lmkIndex).initial_value));
                    else
                        newValues.update(currentLmkKey,Point3(trackedlmks(lmkIndex).initial_value));
                    end
                end
            end
        end
        
        % Update solver
        % =====================================================================
        % We accumulate 2*GPSskip GPS measurements before updating the solver
        % at first so that the heading becomes observable.
        if measurementIndex > firstRTSLAMPose + 2*RTSLAMskip
            isam.update(newFactors, newValues);
            newFactors = NonlinearFactorGraph;
            newValues = Values;
            
            if adding_euc_lmks
                for lmkIndex = eucLmkIndexes
                    if ~trackedlmks(lmkIndex).being_updated
                        trackedlmks(lmkIndex).being_updated = true;
                    end
                end
            end
            
            if rem(measurementIndex,10)==0 % plot every 10 time steps
                cla;
                plot3DTrajectory(isam.calculateEstimate, 'g-');
                title(['Estimated trajectory using ISAM2 (IMU+RTSLAM data) [' int2str(measurementIndex) ']'])
                xlabel('x [m]')
                ylabel('y [m]')
                zlabel('z [m]')
                axis equal
                drawnow;
                M(movie_index) = getframe(gcf);
                movie_index = movie_index + 1;
            end
        % =====================================================================
%             if ~isamPlotted
%                plotBayesTree(isam)
%                isamPlotted = true;
%             end
            currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
            currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
            currentBias = isam.calculateEstimate(currentBiasKey);
        end
    end
end

disp('-- Reached end of sensor data')

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
