% MAIN.m
%
% This script runs trajectory optimization using orthogonal collocation
clc; clear;


% Number of points for initialization:
config.grid.nTrajPts = 10;

% Options for nlp solver (fmincon)
config.options.nlp = optimset(...
    'Display','iter',...
    'MaxIter',250,...
    'MaxFunEvals',1e4);

% Physical parameters for dynamics
m1 = 1.0; config.dyn.m1 = m1;   %cart mass
m2 = 1.0; config.dyn.m2 = m2;   %pendulum mass
config.dyn.g = 9.81;
config.dyn.l = 1;

% Compute an initial guess at a trajectory:
config.guess = computeGuess(config);

% Bounds:
config.bounds = computeBounds(config);

% Create function handles to be called by optimization:
config.function.dynamics = @(t,z,u)( cartPoleDynamics(z,u,config.dyn) );
config.function.costIntegrand = @(t,z,u)( costFunction(t,z,u) );

% Compute the optimal trajectory:
traj = orthogonalCollocation(config);

% Plot the results:
figure(101); clf; plotTraj(traj,config);