clear; clc;

import gtsam.*

% Data Options
options.trajectory_length = 120;
options.nrCameras = 10;
options.lmk_offset = 20;
options.lmk_trajectory_lenght = 120;
options.lmk_step = 10;
options.lmk_distance = 10;
options.square = true;
options.showImages = false;

% iSAM Options
options.hardConstraint = false;
options.pointPriors = false;
options.batchInitialization = true;
options.reorderInterval = 10;
options.alwaysRelinearize = false;

% Display Options
options.saveDotFile = false;
options.printStats = false;
options.drawInterval = 2;
options.cameraInterval = 1;
options.drawTruePoses = false;
options.saveFigures = false;
options.saveDotFiles = false;

%% Generate Data Similar to the one from RT-SLAM
[data,truth] = GenerateRTSLAMData(options);

%% Reduce Data to keyframes

% hold on
% for i = 1:length(truth.points)
%     gtsam.plotPoint3(truth.points{i},'g');
% end
% hold off

%% Initialize iSAM with the first pose and points
[noiseModels,isam,result,nextPose] = VisualISAMInitialize(data,truth,options);
cla;
VisualISAMPlot(truth, data, isam, result, options)

%% Main loop for iSAM: stepping through all poses
for frame_i=3:options.nrCameras
    [isam,result,nextPose] = VisualISAMStep(data,noiseModels,isam,result,truth,nextPose);
    if mod(frame_i,options.drawInterval)==0
        VisualISAMPlot(truth, data, isam, result, options)
    end
end