import gtsam.*;
disp('Example of application of ISAM2 for visual-inertial localization using RTSLAM data')

%% Define options for the example
options.datapath = '/home/mellon/workspace/2012-10-17_caylus-rtslam';
options.rtslamlogpath = '/home/mellon/devel/var/rtslam_to_g2o/log5';
options.similarity_threshold = 0.3;

%% Read data
% Configuration files
disp('-- Reading configuration files from RTSLAM dataset')
setupconfig = ReadConfigFile([options.datapath '/setup.cfg']);
estimationconfig = ReadConfigFile([options.datapath '/estimation.cfg']);

% IMU data
disp('-- Reading MTI sensor data from RTSLAM dataset')
IMU_data = LoadMTIRTSLAMLog([options.datapath '/MTI.log']);

% Image timestamps
disp('-- Reading image timestamps from RTSLAM dataset')
CAMERA_data = LoadImageTimestamps(options.datapath);
firstRTSLAMData = 1;

% Load start time
disp('-- Reading start time from RTSLAM dataset')
fileID = fopen([options.datapath '/sdate.log']);
start_date = fscanf(fileID,'%f');
fclose(fileID);

% RT-SLAM data
% Landmarks data from RTSLAM
disp('-- Reading log from RTSLAM run')
logfiles = dir(fullfile(options.rtslamlogpath,'*.log'));
fulllogfiles = cellfun(@(S) fullfile(options.rtslamlogpath, S),{logfiles(1:end).name}, 'Uniform', 0);
RTSLAM_data = arrayfun(@(x) ReadRTSLAMLog(x),fulllogfiles);

%% Initialization
RTSLAMskip = 350; % Skip this many RTSLAM data before start optimizing. Guarantees the robot moved enough before optimizing (adapted to caylus dataset)

currentPoseGlobal = Pose3(Rot3, Point3); % initial pose is at origin. CHECK: Is it ok?
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1)); % CHECK: Is it ok?
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]); % CHECK: what does this precision mean?
sigma_init_v = noiseModel.Isotropic.Sigma(3, str2double(setupconfig.UNCERT_VLIN)); % CHECK: GTSAM example had a sigma of 1000.0, while RT-SLAM has a value of 0.
sigma_init_b = noiseModel.Isotropic.Sigmas([ str2double(setupconfig.UNCERT_ABIAS) * ones(3,1); str2double(setupconfig.UNCERT_WBIAS) * ones(3,1) ]);
sigma_between_b = [ str2double(setupconfig.UNCERT_ABIAS) * ones(3,1); str2double(setupconfig.UNCERT_WBIAS) * ones(3,1) ]; % CHECK: what's sigma_between_b?! Which value he should give to it?
g = [0;0;-9.8];
w_coriolis = [0;0;0];

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

%% Main Loop
IMUtimes = [IMU_data.Time];

disp('-- Starting main loop')

t_previous = start_date;
% Loop through the images
for measurementIndex = firstRTSLAMData:length(RTSLAM_data)
    
    % At each non=IMU measurement we initialize a new node in the graph
    currentPoseKey = symbol('x',measurementIndex);
    currentVelKey =  symbol('v',measurementIndex);
    currentBiasKey = symbol('b',measurementIndex);
    t = RTSLAM_data(measurementIndex).r.date;

    CurrentRTSLAMPosition = Point3(RTSLAM_data(measurementIndex).r.pose_mean(1:3));
    CurrentRTSLAMRot = Rot3(quat2dcm(RTSLAM_data(measurementIndex).r.pose_mean(4:7)'));

    if measurementIndex == firstRTSLAMData
        %% Create initial estimate and prior on initial pose, velocity, and biases
        newValues.insert(currentPoseKey, currentPoseGlobal);
        newValues.insert(currentVelKey, currentVelocityGlobal);
        newValues.insert(currentBiasKey, currentBias);
        newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
        newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
        newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
    else
        % Integrate IMU measurements until the time of the current image
        IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
        t_previous = t;
                
        currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
            currentBias, ...
            (str2double(setupconfig.PERT_AERR)*str2double(setupconfig.ACCELERO_NOISE)).^2 * eye(3), ...
            (str2double(setupconfig.PERT_WERR)*str2double(setupconfig.GYRO_NOISE)).^2 * eye(3), ...
            0.^2 * eye(3)); % CHECK: Integration noise set to 0. Is that OK?
        
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
        RTSLAMPose = Pose3(CurrentRTSLAMRot, CurrentRTSLAMPosition);
%        GPSPose = Pose3(currentPoseGlobal.rotation, GPS_data(measurementIndex).Position);
%         if mod(measurementIndex, GPSskip) == 0 %CHECK: Add GPS measurements each GPSskip nodes
%             newFactors.add(PriorFactorPose3(currentPoseKey, GPSPose, noiseModelGPS));
%         end
        
        % Add initial value
        newValues.insert(currentPoseKey, RTSLAMPose); % CHECK: Values from RT-SLAM would enter here.
        newValues.insert(currentVelKey, currentVelocityGlobal);
        newValues.insert(currentBiasKey, currentBias);
        
        % Update solver
        % =====================================================================
        % We accumulate 2*GPSskip GPS measurements before updating the solver
        % at first so that the heading becomes observable.
        if measurementIndex > firstRTSLAMData + RTSLAMskip
            isam.update(newFactors, newValues);
            newFactors = NonlinearFactorGraph;
            newValues = Values;
            
            if rem(measurementIndex,10)==0 % plot every 10 time steps
                cla;
                plot3DTrajectory(isam.calculateEstimate, 'g-');
                title('Estimated trajectory using ISAM2 (IMU+RTSLAM data)')
                xlabel('[m]')
                ylabel('[m]')
                zlabel('[m]')
                axis equal
                drawnow;
            end
        % =====================================================================
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
