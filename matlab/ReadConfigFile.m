function R = ReadConfigFile(configfile)
% READCONF reads configuration file. 
%
% The value of boolean parameters can be tested with 
%    exist(parameter,'var')
 
fid = fopen(configfile); 
if fid<0, error('cannot open file %s\n',a); end; 
 
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line) || all(isspace(line)) || strncmp(line,'#',1) || strncmp(line,';',1),
	; % no operation 
    else 
	[var,tok] = strtok(line,' \t=:');
    % remove separators from token
    tok = strrep(tok,' ','');
    tok = strrep(tok,'\t','');
    tok = strrep(tok,'=','');
    tok = strrep(tok,':','');
    % trim tok
    tok = strtrim(tok);    
	R.(var) = tok;		% return value of function
%	eval(sprintf('%s=''%s''; ',var,tok));  % stores variable in local workspace
    end;
end; 
fclose(fid);
%whos,     % shows the parameter in the local workspace 