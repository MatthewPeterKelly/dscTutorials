%MAIN_cannon.m
%
% This script is used to find the optimal initial state for the cannon
% example problem, as described below:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Cannon Example:
%
% Find the optimal initial velocity for a projectile such that the
% resulting trajectory passes through a desired target. 
%
% States:
%   x = horizontal position
%   y = vertical position
%   dx = horizontal speed
%   dy = vertical speed
%   ddx = horizontal acceleration
%   ddy = vertical acceleration
%
% Parameters:
%   c = quadratic drag coefficient
%   T = final simulation time
%
% Boundary Conditions:
%   x(0) = 0;
%   y(0) = 0;
%   x(T) = xf;
%   y(T) = yf;
%
% Objective Function:
%   J = dx(0)^2 + dy(0)^2;
%
% Dynamics Constraint:
%   ddx = -c*dx*sqrt(dx.*dx + dy.*dy);
%   ddy = -c*dy*sqrt(dx.*dx + dy.*dy) - 1; 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Provide an initial guess for the solver
guess.initSpeed = 9.0;
guess.initAngle = (pi/180)*45;

% Set the target (assuming that the trajectory starts at x=0, y=0)
target.x = 6.0;
target.y = 0;

% Set up the problem parameters:
param.c = 0.4;  %Quadratic drag coefficient

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param.nGrid = 26;  %Discritization of the solver grid

%%% Parameters for problem diagnostics
param.diagnostics.enable = true;   %Enable plotting and log iterations?
param.diagnostics.plotPause = 0.25;  %How much to pause on each plot iteration
param.diagnostics.writeGif = true;   %Save each iteration to a gif
param.diagnostics.gifName = 'cannon_singleShooting.gif';
param.diagnostics.gifPixelDim = [800,400];  %How big of a gif to make?

% Use single shooting to find the answer:
soln = cannon_singleShooting(guess,target,param);

% Plot the solution:
figure(101); clf; plotTraj(soln, target);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param.nSegment = 5;
param.nSubStep = 5;

%%% Parameters for problem diagnostics
param.diagnostics.enable = true;   %Enable plotting and log iterations?
param.diagnostics.plotPause = 0.25;  %How much to pause on each plot iteration
param.diagnostics.writeGif = true;   %Save each iteration to a gif
param.diagnostics.gifName = 'cannon_multipleShooting.gif';
param.diagnostics.gifPixelDim = [800,400];  %How big of a gif to make?

% Use single shooting to find the answer:
soln = cannon_multipleShooting(guess,target,param);

% Plot the solution:
figure(103); clf; plotTraj(soln, target);









