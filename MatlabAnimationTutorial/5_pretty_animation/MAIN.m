% MAIN.m  --  Lesson 5 -- Pretty Animation
%
% This script performs an simulation and animation of a cart-pole as it
% moves passively along a horizontal track.
%
% Graphics improved to include patch objects, rather than simple lines and
% dots.
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
hold on;    %  <-- This is important!
set(gcf,'DoubleBuffer','on');   % Prevents flickering (maybe??)

cartColor = [0.2, 0.7, 0.2];   % [R, G, B]
poleColor = [0.3, 0.2, 0.7];

% Draw the rail that the cart-pole travels on
plot(extents(1:2),[0,0],'k-','LineWidth',2);

% Compute the verticies of a star, just for fun;
star = getStarVerticies(7,0.5);
p.star = 0.6*p.r*star;  %Rescale;


time = 0;
global cartHandle poleHandle bobHandle starHandle
cartHandle = [];
poleHandle = [];
bobHandle = [];
starHandle = [];
tic;
while time < t(end)
    
    % Compute the position of the system at the current real world time
    posDraw = interp1(t',pos',time')';
    
    drawCartPole(time,posDraw,extents,p);
    
    time = toc;
end
