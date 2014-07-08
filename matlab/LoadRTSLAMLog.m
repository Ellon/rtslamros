
%% Load data from RT-SLAM log
RTSLAM_data = importdata('rtslam_orig_edited.log');
RTSLAM_data = cell2struct(num2cell(RTSLAM_data.data), RTSLAM_data.colheaders, 2);

RTSLAM_data2 = importdata('rtslam_orig_edited2.log');
RTSLAM_data2 = cell2struct(num2cell(RTSLAM_data2.data), {RTSLAM_data2.textdata{1,2:end}}, 2);
