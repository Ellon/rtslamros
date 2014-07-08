function [data] = LoadImageTimestamps(datapath)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LoadImageTimestamps loads the image timestamps from the *.time files on 
% RTSLAM dataset. For the i-th image, the funcion returns the image 
% filename in data[i].filename and the timestamp into data[i].timestamp
%
% Author: Ellon P. Mendes <ellonpaiva@gmail.com>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

times = dir([datapath '/*.time']); %Get all .time files
for i=1:length(times)
    fileID = fopen([datapath '/' times(i).name]);
    data(i).Time = fscanf(fileID,'%f');
    fclose(fileID);
    data(i).name = [times(i).name(1:end-5) '.png'];
end
