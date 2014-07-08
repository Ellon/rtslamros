function v = ParseUblasVector(vectorstr)
% PARSEUBLASVECTOR parse a string containing a ublas vector to matlab format. 
% The string should be in the format:
% [<vector_size>](<value_1>,<value_2>,...,<value_n>)
% 
% Author: Ellon P. Mendes <ellonpaiva@gmail.com.

% look for the header 
matches = regexp(vectorstr,{'\[[0-9]*\]'},'match');
header = cell2mat(matches{1});

% get vector size
size = str2double(header(2:end-1));

% strip out the header
body = strrep(vectorstr,header,'');

% split body
terms = strsplit(body(2:end-1),',');

% convert from str to double
v = str2double(terms);

% TODO Maybe it will be interesting to do a size test here, to check if the
% UBLAS string is valid.
