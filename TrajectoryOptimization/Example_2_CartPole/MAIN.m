% MAIN.m
%
% This script runs trajectory optimization using either orthogonal
% collocation or direct transcription (trapazoid).
clc; clear;


% Number of points for initialization:
config.grid.nTrajPts = 15;

% Options for nlp solver (fmincon)
config.options.nlp = optimset(...
    'Display','iter',...
    'MaxIter',250,...
    'MaxFunEvals',5e4);

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

%%%% Select method and compute trajectory
% traj = orthogonalCollocation(config);
traj = dirTrans_Trapz(config);

% Animation:
P.plotFunc = @(t,z)( drawCartPole(t,z,config.dyn) );
P.speed = 0.7;
P.figNum = 102;
t = linspace(traj.time(1),traj.time(end),250);
z = traj.interp.state(t);
animate(t,z,P)

% Plot the results:
figure(101); clf; plotTraj(traj,config);