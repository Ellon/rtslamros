function [log] = ReadRTSLAMLog(filename,options)

    if ~exist('options','var')
        % set default options for the function
        options.read_lmk_events = true;
        options.read_lmk_params = true;
        options.read_lmk_expectation = false;
        options.read_lmk_measurement = true;
        options.read_lmk_innovation = false;
    else
        % check if all suboptions exists, if not set the default
        if ~isfield(options,'read_lmk_events')
            options.read_lmk_events = true;
        end
        if ~isfield(options,'read_lmk_params')
            options.read_lmk_params = true;
        end
        if ~isfield(options,'read_lmk_expectation')
            options.read_lmk_expectation = false;
        end
        if ~isfield(options,'read_lmk_measurement')
            options.read_lmk_measurement = true;
        end
        if ~isfield(options,'read_lmk_innovation')
            options.read_lmk_innovation = false;
        end
    end
    
    if iscell(filename)
        filename = cell2mat(filename);
    end
    
    % open the file
    fid = fopen(filename);
        
    % read the robot
	log.r = LoadRobotLog(fid);
    
	% read the number of landmarks
    n_lmks = fread(fid, 1, 'uint64');
    
    % read each of the landmarks
	for i = 1:n_lmks
		log.lmks(i) = LoadLandmarkLog(fid,options);
	end
    
    % close the file
    fclose(fid);
end

function robot_log = LoadRobotLog(fid)
    robot_log.index = fread(fid, 1, 'uint64');
    robot_log.date = fread(fid, 1, 'double');
    robot_log.pose_mean = fread(fid, 7, 'double');
    robot_log.pose_cov = fread(fid, [7 7], 'double');
end

function lmk_log = LoadLandmarkLog(fid,options)
    lmk_log.id = fread(fid, 1, 'uint64');
    lmk_log.type = fread(fid, 1, 'int');
    
    if options.read_lmk_events
        lmk_log.ev_predicted = fread(fid, 1, 'uint8');
        lmk_log.ev_visible = fread(fid, 1, 'uint8');
        lmk_log.ev_measured = fread(fid, 1, 'uint8');
        lmk_log.ev_matched = fread(fid, 1, 'uint8');
        lmk_log.ev_updated = fread(fid, 1, 'uint8');
        lmk_log.c_nSearch = fread(fid, 1, 'int');
        lmk_log.c_nMatch = fread(fid, 1, 'int');
        lmk_log.c_nInlier = fread(fid, 1, 'int');
        lmk_log.c_nSearchSinceLastInlier = fread(fid, 1, 'int');
        lmk_log.c_nFramesSinceLastVisible = fread(fid, 1, 'int');
    else
        fseek(fid, 25, 'cof');
    end
    
    if lmk_log.type == 0 % euclid lmk
        mean_size = 3;
        cov_size = [3 3];
    else % ahp lmk
        mean_size = 7;
        cov_size = [7 7];
    end
    
    if options.read_lmk_params
        lmk_log.mean = fread(fid, mean_size, 'double');
        lmk_log.cov = fread(fid, cov_size, 'double');
    else
        fseek(fid, mean_size*8 + prod(cov_size)*8, 'cof');
    end

    if options.read_lmk_expectation
        lmk_log.exp_mean = fread(fid, 2, 'double');
        lmk_log.exp_cov = fread(fid, [2 2], 'double');
    else
        fseek(fid, 2*8 + prod([2 2])*8, 'cof');
    end
    
    if options.read_lmk_measurement
        lmk_log.meas_mean = fread(fid, 2, 'double');
        lmk_log.meas_cov = fread(fid, [2 2], 'double');
    else
        fseek(fid, 2*8 + prod([2 2])*8, 'cof');
    end
    
    if options.read_lmk_innovation
        lmk_log.innov_mean = fread(fid, 2, 'double');
        lmk_log.innov_cov = fread(fid, [2 2], 'double');
    else
        fseek(fid, 2*8 + prod([2 2])*8, 'cof');
    end
   
end


