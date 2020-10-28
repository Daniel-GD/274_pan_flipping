% Matlab needs to know where all your functions are, so we add them to the
% Matlab path.  Unless the changes are saved, they will be removed from the
% path when Matlab closes. This is good, because we don't want folders on
% the path that we aren't using, else we run the risk of using files of the
% same name but with different contents.

% pwd is the Present Working Directory
addpath([pwd '/Modeling'])
addpath([pwd '/Modeling/Arm'])
addpath([pwd '/Modeling/Pancake'])
% addpath([pwd '/Visualization'])
addpath([pwd '/AutoDerived'])
addpath([pwd '/AutoDerived/Arm'])
addpath([pwd '/AutoDerived/Pancake'])

addpath([pwd '/Simulation'])
addpath([pwd '/Simulation/Arm'])
addpath([pwd '/Simulation/Pancake'])
% addpath([pwd '/Optimization'])
addpath([pwd '/Visualization'])
addpath([pwd '/Misc'])