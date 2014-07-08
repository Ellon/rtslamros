function [data,truth] = GenerateRTSLAMData(options)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GenerateRTSLAMData generates simulated data to be used as RT-SLAM data
% The robot goes in a straight line. Landmarks are positioned around robot
% trajectory. The robot camera will detect landmarks in its field of view,
% dividing them into close landmarks and distant landmarks according to a
% ratio.
% 
% Author: Ellon P. Mendes <ellonpaiva@gmail.com>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Generate simulated landmarks
if options.square % Create a triangle target, just 3 points on a plane
    nrPointPivots = floor(options.lmk_trajectory_lenght/options.lmk_step);
    nrPoints = nrPointPivots*4;
    r = options.lmk_distance;
    for i=1:nrPoints
        theta = mod(i-1,4)*pi/2;
        pivot = options.lmk_offset + floor((i-1)/4)*options.lmk_step;
        truth.points{i} = Point3(pivot, r*sin(theta), r*cos(theta));
    end
else % 3D landmarks as vertices of a cube
    nrPoints = 8;
    truth.points = {Point3([10 10 10]'),...
        Point3([-10 10 10]'),...
        Point3([-10 -10 10]'),...
        Point3([10 -10 10]'),...
        Point3([10 10 -10]'),...
        Point3([-10 10 -10]'),...
        Point3([-10 -10 -10]'),...
        Point3([10 -10 -10]')};
end

%% Create camera cameras on a line through the points
truth.K = Cal3_S2(500,500,0,640/2,480/2);
data.K = truth.K;
for i=1:options.nrCameras
    t = Point3([(i-1)*options.trajectory_length/options.nrCameras, 0, 0]');
    % This offset is needed to allow the projections below. Should be
    % removed when correcting dealing with the projections
    t = t.compose(Point3([-20,0,0]')); 
    truth.cameras{i} = SimpleCamera.Lookat(t, t.compose(Point3([10,0,0]')), Point3([0,0,1]'), truth.K);
    % Create measurements
    Z_j = 1;
    for j=1:nrPoints
        % All landmarks seen in every frame
        [Z,proj_ok] = truth.cameras{i}.projectSafe(truth.points{j});
        if  proj_ok && Z.x() >=0 && Z.x() <640 && Z.y() >=0 && Z.y() <480   
            data.Z{i}{Z_j} = Z;
            data.J{i}{Z_j} = j;
            Z_j = Z_j + 1;
        end
    end    
end

%% show images if asked
if options.showImages
    gui = gcf;
    for i=1:options.nrCameras
        figure(2+i);clf;hold on
        set(2+i,'NumberTitle','off','Name',sprintf('Camera %d',i));
        for j=1:nrPoints
            zij = truth.cameras{i}.project(truth.points{j});
            plot(zij.x,zij.y,'*');
            axis([1 640 1 480]);
        end
    end
    figure(gui);
end

%% Calculate odometry between cameras
odometry = truth.cameras{1}.pose.between(truth.cameras{2}.pose);
for i=1:options.nrCameras-1
    data.odometry{i}=odometry;
end

% %% Create one camera to make iSAM happy 
% truth.K = Cal3_S2(500,500,0,640/2,480/2);
% data.K = truth.K;
% for i=1:options.nrCameras
%     t = Point3([0, 0, 0]');
%     truth.cameras{i} = SimpleCamera.Lookat(t, Point3, Point3([0,0,1]'), truth.K);
%     % Create measurements
%     for j=1:nrPoints
%         % All landmarks seen in every frame
%         data.Z{i}{j} = truth.cameras{i}.project(truth.points{j});
%         data.J{i}{j} = j;
%     end    
% end
% 
% %% Generate robot trajectory
% 
% %% Generate landmarks measurements
% 
% %% Generate robot odometry