import gtsam.*;
disp('Example of application of ISAM2 for visual-inertial localization using RTSLAM data')

%% Define options for the example
options.logroot = '/home/emendes/devel/var/rtslam_to_g2o/log5';
options.similarity_threshold = 0.3;

%% Read data
disp('-- Reading MTI sensor data from file')
% IMU data
IMU_data = LoadMTIRTSLAMLog('MTI.log');

disp('-- Reading data from RTSLAM log files')
% Landmarks data from RTSLAM
logfiles = dir(fullfile(options.logroot,'*.log'));
RTSLAM_data = arrayfun(@(x) ReadRTSLAMLog(x),fullfile(options.logroot,{logfiles(1:end).name}));

current_keyframe = RTSLAM_data(1);
plot(current_keyframe.r.pose_mean(1),current_keyframe.r.pose_mean(2),'x');
for data = RTSLAM_data(1:end)
    if(IsKeyframe([current_keyframe.lmks.id],[data.lmks.id],options))
        hold on
        plot(data.r.pose_mean(1),data.r.pose_mean(2),'x');
        hold off
        current_keyframe = data;
    end
end