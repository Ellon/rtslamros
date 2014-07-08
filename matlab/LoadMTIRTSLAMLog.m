function [data] = LoadMTIRTSLAMLog(filename)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LoadMTIRTSLAMLog loads MTI data from a file which was saved by RT-SLAM,
% Data in the file should be in the format defined by jblas::vector>>
% method, and in the following order:
%    Time accelX accelY accelZ omegaX omegaY omegaZ magneX magneY magneZ
%
% Author: Ellon P. Mendes <ellonpaiva@gmail.com>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fileID = fopen(filename);
data = textscan(fileID,'[10](%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)');
fclose(fileID);
data = cell2struct(num2cell(cell2mat(data)), ...
                   {'Time','accelX','accelY','accelZ', ... 
                   'omegaX','omegaY','omegaZ', ...
                   'magneX','magneY','magneZ'}, 2);
data(1).dt = data(1).Time;
for i=2:length(data)
    data(i).dt = data(i).Time - data(i-1).Time;    
end
