function [trackedlmks,lmkmap] = TrackRTSLAMLmks(log,trackedlmks,lmkmap,robot_pose_key)

for i=1:numel(log.lmks)
    lmk_key = log.lmks(i).id;
    
    %if(lmk_index > numel(trackedlmks)) 
    if ~lmkmap.isKey(lmk_key)
        % Lmk never seen before: initialize it
        lmk_index = numel(trackedlmks) + 1;
        
        lmkmap = [lmkmap; containers.Map(lmk_key,lmk_index)];
        trackedlmks( lmk_index ).being_updated = false;
        trackedlmks( lmk_index ).meas = [];
        trackedlmks( lmk_index ).robot_pose_key = uint64([]);
    else
        lmk_index = lmkmap(lmk_key);
    end
    
    if numel(trackedlmks( lmk_index ).meas) == 0 % Lmk seen but previous measurements were used
        n_meas = 0;
    else % Lmk seen and currently has measurements in the buffer
        n_meas = size(trackedlmks( lmk_index ).meas,3);    
    end
    
    % only use the measurement if the landmark was matched in the image
    if log.lmks(i).ev_matched
        trackedlmks( lmk_index ).meas(:,:,n_meas+1) = log.lmks(i).meas_mean;
        trackedlmks( lmk_index ).robot_pose_key(n_meas+1) = robot_pose_key;
    end
    
    if(log.lmks(i).type == 0)
        trackedlmks( lmk_index ).is_euclidean = true;
        trackedlmks( lmk_index ).initial_value = log.lmks(i).mean;
    else
        trackedlmks( lmk_index ).is_euclidean = false;
    end
end

