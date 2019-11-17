%% ------------------------- Add path -----------------------------
addpath(genpath('GameDefine'));
addpath(genpath('ReachableSet'));
addpath(genpath('utils'));
addpath(genpath('ToolboxLS'));

%% ------------------- Initial game setting -----------------------
% defender and attacker initial position
gameParam.xa_init{1} = [-0.3 0.2];
gameParam.xd_init{1} = [0.11 0.];
gameParam.xa_init{2} = [0.05 -0.2];
gameParam.xd_init{2} = [0.11 0];

% define player velocity
gameParam.va = 1;
gameParam.vd = 1;

% define capture radius
gameParam.captureRadius = 0.05;

% define backward compute time
gameParam.timeMax = 1;

% define resolution
gameParam.Nx = 41;

% - Define game type
% - 'protect'  the defender just try to keep attacker away from the target
% - 'chase'    the defender tries to catch the attacker

gameParam.type = 'protect';
% gameParam.type = 'chase';

%% ------------------------- run game -----------------------------
% [data] = computeReachableSet(gameParm, 'medium', 'circleTarget_noObs');
% [data] = computeReachableSet(gameParm, 'low', 'nonconvexExample');
[data, g, dataset, figStruct] = computeReachableSet(gameParam, 'low', 'CircleObsAndTarget');

% compare with geometric method
hold on;
winning_region(0, 0.1);

%% -------------------- Optimal trajectory ------------------------
dataset_f = flip(dataset, 5);
[traj, traj_tau] = OptTrajCal(g, dataset_f, figStruct.tau, gameParam);

%% -------------------- Plot trajectory ---------------------------
plotOptTraj(figStruct, traj, traj_tau);


