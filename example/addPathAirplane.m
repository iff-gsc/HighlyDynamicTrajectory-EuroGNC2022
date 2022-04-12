function [] = addPathAirplane()
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% go to correct directory
init_script_path = fileparts(mfilename('fullpath'));
this_workbench_path = fileparts(init_script_path);
cd(init_script_path);

% add path for initialization
% go to correct directory
init_scripts_path = fileparts(mfilename('fullpath'));
cd(init_scripts_path);
cd ..

% add folders to path
addpath(genpath('libraries'));
addpath(genpath('models'));

% enter workbench
cd(this_workbench_path);

% add folders to path
addpath(genpath(this_workbench_path));

end