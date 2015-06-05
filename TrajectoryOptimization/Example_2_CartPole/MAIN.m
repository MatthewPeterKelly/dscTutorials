% MAIN.m
%
% This script runs trajectory optimization using orthogonal collocation

% Number of points for initialization:
config.grid.nTrajPts = 11;

% Options for nlp solver (fmincon)
config.options.nlp = optimset(...
    'Display','iter',...
    'MaxIter',100,...
    'MaxFunEvals',1e4);

% Physical parameters for dynamics
m1 = 1.0; config.dyn.m1 = m1;   %cart mass
m2 = 1.0; config.dyn.m2 = m2;   %pendulum mass
config.dyn.g = 9.81;
config.dyn.l = 1;

% Create function handles to be called by optimization:
config.function.dynamics = @(t,z,u)( cartPoleDynamics(z,[u;zeros(size(u))],config.dyn) );
config.function.pathCost = @(t,z,u)( costFunction(t,z,u) );
config.function.endpointCost = [];
config.function.pathConstraint = [];
config.function.endpointConstraint = @(t,z0,zF)( boundaryConstraint(t,z0,zF,config.bndCst) );

% Compute an initial guess at a trajectory:
config.guess = computeGuess(config);

% Bounds:
[config.bounds, config.userData] = computeBounds(config);

% Compute the optimal trajectory:
traj = orthogonalCollocation(config);

% Plot the results:
figure(101); clf; plotTraj(traj,config);