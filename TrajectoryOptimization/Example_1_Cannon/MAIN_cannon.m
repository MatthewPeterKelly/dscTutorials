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
clc; clear;

%Provide an initial guess for the solver
guess.initSpeed = 9.0;
guess.initAngle = (pi/180)*45;

% Set the target (assuming that the trajectory starts at x=0, y=0)
target.x = 6.0;
target.y = 0;

% Set up the parameters for the dynamics function:
param.dynamics.c = 0.4;  %Quadratic drag coefficient

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Set up the grid discretization for each method:
param.singleShooting.nGrid = 24; 
param.multipleShooting.nSegment = 8;
param.multipleShooting.nSubStep = 3;
param.collocation.nGrid = 24;

%%% Parameters for diagnostics (visualization only)
param.diagnostics.enable = true;   %Enable plotting and log iterations?
param.diagnostics.animationDuration = 8;  %(seconds) How long is the animation?
param.diagnostics.writeGif = false;   %Save animation to a gif?
param.diagnostics.gifPixelDim = [800,400];  %How big of a gif to make?
param.diagnostics.figNum.singleShooting = 10;
param.diagnostics.figNum.multipleShooting = 11;
param.diagnostics.figNum.collocation = 12;

% % Use single shooting to find the solution:
% soln.singleShooting = cannon_singleShooting(guess,target,param);
% 
% % Use multiple shooting to find the solution:
% soln.multipleShooting = cannon_multipleShooting(guess,target,param);

% Use collocation to find the solution:
soln.collocation = cannon_collocation(guess,target,param);









