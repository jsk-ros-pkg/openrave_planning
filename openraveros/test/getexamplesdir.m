%% exampledir=addexamplesdir(subdir)
%%
%% Adds the openrave examples directory
function exampledir=getexamplesdir(subdir)

basepath = [];
if( isunix() )
    %% try getting from openrave-config
    [status,basepath] = system('openrave-config --prefix');
    basepath = strtrim(basepath);
end

if( isempty(basepath) )
    basepath = 'C:\Program Files\openrave';
end

if( ~exist('subdir','var') )
    subdir = [];
end

exampledir=fullfile(basepath,'share','openrave','examples',subdir);
