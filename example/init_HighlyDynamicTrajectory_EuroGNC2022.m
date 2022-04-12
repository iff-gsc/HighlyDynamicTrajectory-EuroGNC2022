% Reference simulation for the paper published at the EuroGNC 2022
% conference:
%
%       Accurate tracking of highly dynamic airplane
%     trajectories using incremental nonlinear dynamic
%                     inversion

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Fabian Guecker
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Clear workspace and console
addPathAirplane();
clc_clear;

%% Create waypoints
% Easy Eight
% waypoints = [1 -1 0.2; 2 0 -0.5; 1 1 0; -1 -1 0; -2 0 -0.5; -1 1 0.2]'*15;
% cycle = true;

% Crazy Eight
waypoints = [1 -1 0.2; 2 0 -1.5; 1 1 0; -1 -1 0; -2 0 -0.5; -1 1 0.2]'*15;
cycle = true;

% Straight Line
% waypoints = [-1 0 0; 0 0 0; 1 0 0; 2 0 0; 3 0 0; 4 0 0; 5 0 0; 6 0 0]'*200;
% cycle = false;

% Circle
% waypoints = [ 1 1 0; 1 -1 0; -1 -1 0; -1 1 0;]'*30;
% cycle = true;

%% Generate empty trajectory struct and bus definition for simulink 

% Degree of polynominals
degree = 5;

% Create empty trajectory with size of waypoints
traj_size = size(waypoints, 2);
[traj_empty] = trajInit(traj_size, degree);

% Create simulink bus definition
trajectoryBus = struct2bus_(traj_empty);

%% Compute trajectory

traj = trajInit(traj_size, degree);
traj = trajFromWaypoints(traj, waypoints, degree, cycle);

%% Calculate inital values for steady-state start of simulation

% inital and target velocity for motorglider 10-20 m/s
inital_velo  = 20;
g = 9.81;

% inital position is the first waypoint
inital_point = waypoints(:,1);

% inital velocity vector and acceleration
% Analytic solution
%[active_section, Error, t] = trajGetMatch(traj, inital_point);
% Iterative solution
[inital_section, Error, t] = trajGetMatchEnhanced(traj, inital_point);

% Calculate reference attitude
traj_section = trajGetSection(traj, inital_section);
[T, B, N] = trajSectionGetFrenetSerretWithGravity(traj_section,inital_velo, g, t);
inital_vel_vec = inital_velo * T;

% Calculate inital attitude
initalSerretFrame = [T, B, -N];
inital_quaternion = rotm2quat(initalSerretFrame);

%% Plot trajectory with expected acceleration for constant velocity

figure(1);
trajPlot(traj, inital_velo, g);
view([-1 -1.2 0.5])
axis equal
grid on

%% init airplane parameters
% init airplane parameters
airplane = conventionalAirplaneLoadParams( ...
	'params_conventional_airplane_EasyGlider' );

% environment parameters
envir = envirLoadParams( 'envir_params_default', 'envir', 0 );

% Updated inital parameters
airplane.posRef.alt = 50;

% FlightGear Visualisation / Iceland
%airplane.posRef.lat = 63.985024;
%airplane.posRef.lon = -22.597652;

% FlightGear Visualisation / Honolulu
airplane.posRef.lat = 21.325229677012764;
airplane.posRef.lon = -157.93891162805602;

% Update inital conditons to active trajectory
airplane.ic.q_bg = inital_quaternion';
airplane.ic.V_Kb = [norm(inital_vel_vec); 0; 0];
airplane.ic.s_Kg = inital_point;
airplane.ic.omega_Kb = [0; 0; 0];

% Control effectiveness at 14 m/s for motorglider model
B_aileron_p_14ms        =  95.35;
B_elevator_q_14ms       =  59.53;
B_coll_aileron_q_14ms   = -39.27;
B_rudder_r_14ms         =  17.66;
B_elevator_w_14ms       =  4.178;
B_coll_aileron_w_14ms   = -34.8;

% INDI Attitude Controller
K_atti = ndiFeedbackGainPlace( -10*ones(3,1), 2/80 + 2/100 );

% Position Controller Vertical-Channel
K_vertical = ndiFeedbackGainPlace(-2.0*ones(3,1), 0); 

% Position Controller Lateral-Channel
K_lateral = ndiFeedbackGainPlace( -0.75*ones(3,1), 0);

%% Control Allocation
% k is the number of control inputs (actutators outputs)
% m is the number of pseudo control inputs

% minimum control input kx1 vector
ca.u_min = -ones(4,1);
% maximum control input kx1 vector
ca.u_max = ones(4,1);
% desired control input kx1 vector
ca.u_d = zeros(4,1);
% weighting mxm matrix of pseudo-control
ca.W_v = diag([10, 10, 10, 1]);
% weighting kxk matrix of the control input vector
ca.W_u = diag([1e4, 1e4, 1, 1]);
% weighting of pseudo-control vs. control input (scalar)
ca.gamma = 1e6;
% initial working set mx1 vector
ca.W = zeros(4,1);
% maximum number of iterations (scalar)
ca.i_max = 100;

% control mxk matrix B for an airplane
%  X : major influence
% (X): minor influence
%  - : influence can be neglected
% 
%    ail_l, ail_r, ele, rud,
%p     X     X      -    -    
%q     X     X      X    -   
%r    (X)   (X)     -    X   
%z     X     X     (X)   - 

% control mxk matrix B, hover
ca.B = [B_aileron_p_14ms/2, -B_aileron_p_14ms/2, 0, 0;
        B_coll_aileron_q_14ms/2, B_coll_aileron_q_14ms/2, B_elevator_q_14ms, 0;
        0, 0, 0, B_rudder_r_14ms;
        B_coll_aileron_w_14ms/2, B_coll_aileron_w_14ms/2, B_elevator_w_14ms, 0,];


%% open model
open_model('HighlyDynamicTrajectory_EuroGNC2022');
