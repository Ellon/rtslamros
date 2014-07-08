function [is_keyframe, similarity] = IsKeyframe(lmk_indexes1, lmk_indexes2, options)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IsKeyframe takes two lists of indexes an verifies if the second list
% represents a keyframe in relation to the first list. The parameter
% option.similarity_threshold should have been defined. 
% 
% The second list is considered a keyframe if the similarity with the first
% list drops below the defined threshold.
%   
% Author: Ellon P. Mendes <ellonpaiva@gmail.com>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    n_common_lmks = length(intersect(lmk_indexes1, lmk_indexes2));
    % similarity = n_common_lmks/min(length(lmk_indexes1),length(lmk_indexes2));
    similarity = (2*n_common_lmks)/(length(lmk_indexes1)+length(lmk_indexes2));
    is_keyframe = similarity < options.similarity_threshold;
