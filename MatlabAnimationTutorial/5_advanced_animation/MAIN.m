% MAIN.m  --  Lesson 5 -- Advanced Animation
%
% This script performs an simulation and animation of a cart-pole as it
% moves passively along a horizontal track. Keyboard call-backs are added
% to make the simulation interactive.
%

clc; clear;

%%%% Initial State
z0 = [
    0.0;   %horizontal position
    (pi/180)*80;  %pendulum angle (wrt gravity)
    0.3;   %horizontal velocity
    0.0];  %pendulum angular rate

%%%% Physical Parameters  (big mass and inertia for "slow" physics)
p.m1 = 18.0;  % (kg) Cart mass
p.m2 = 12.0;  % (kg) pole mass
p.g = 9.81;  % (m/s^2) gravity 
p.l = 5.0;   % (m) pendulum (pole) length 

%%%% Time vector
t = linspace(0,10,250);  %Simulation time stamps

%%%% Function Handle
dynFun = @(t,z)( cartPoleDynamics(z, p) );

%%%% Simulate the system!
options = odeset(...
    'RelTol',1e-8, ...
    'AbsTol',1e-8);
[~, z] = ode45(dynFun, t, z0, options);   %  <-- This is the key line!
z = z';

%%%% Plots:
figure(1); clf;
plotCartPole(t,z);  %Moved plotting to its own function


%%%% Animation:

% Convert states to cartesian positions:
pos = cartPolePosition(z,p); 
x1 = pos(1,:);
y1 = pos(2,:);
x2 = pos(3,:);
y2 = pos(4,:);

% Compute the extents of the drawing, keeping everything in view
padding = 0.2*p.l;  %Free space around edges
xLow = min(min(x1,x2)) - padding;
xUpp = max(max(x1,x2)) + padding;
yLow = min(min(y1,y2)) - padding;
yUpp = max(max(y1,y2)) + padding;
extents = [xLow,xUpp,yLow,yUpp];

% Set up for a call to animate.m
P.plotFunc = @(t,x)( drawCartPole(t,x,extents) );
P.figNum = 2; 
P.speed = 1.0;  %Animate in real time
animate(t,pos,P);


