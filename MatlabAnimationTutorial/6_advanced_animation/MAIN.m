% MAIN.m  --  Lesson 6 -- Advanced Animation
%
% This script performs an simulation and animation of a cart-pole as it
% moves passively along a horizontal track. Keyboard call-backs are added
% to make the simulation interactive.
%
% This lesson takes a huge leap in complexity, introducing graphics
% call-back functions as well a more sophisticated method for adjusting the
% animation time step, which allows the user to speed up, slow down, and
% pause the animation.
%

clc; clear; clear global;

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
figure(1); clf; hold on;
plotCartPole(t,z);  %Moved plotting to its own function


%%%% Animation:

% Convert states to cartesian positions:
pos = cartPolePosition(z,p);
x1 = pos(1,:);
y1 = pos(2,:);
x2 = pos(3,:);
y2 = pos(4,:);

% Plotting parameters:
p.w = 0.6*p.l;  %Width of the cart
p.h = 0.4*p.l;  %Height of the cart
p.r = 0.1*p.l;  % Radius of the pendulum bob

% Compute the extents of the drawing, keeping everything in view
padding = 0.2*p.l;  %Free space around edges
xLow = min(min(x1 - 0.5*p.w,  x2 - p.r)) - padding;
xUpp = max(max(x1 + 0.5*p.w,  x2 + p.r)) + padding;
yLow = min(min(y1 - 0.5*p.h,  y2 - p.r)) - padding;
yUpp = max(max(y1 + 0.5*p.w,  y2 + p.r)) + padding;
extents = [xLow,xUpp,yLow,yUpp];

% Create and clear a figure:
figure(2); clf;
set(gcf,'DoubleBuffer','on');   % Prevents flickering (maybe??)

% Compute the verticies of a star, just for fun;
star = getStarVerticies(7,0.5);
p.star = 0.6*p.r*star;  %Rescale;

% Set up for a call to animate.m
P.plotFunc = @(t,x)( drawCartPole(t,x,extents,p) );
P.figNum = 2; 
P.speed = 1.0;  %Animate in real time
animate(t,pos,P);


