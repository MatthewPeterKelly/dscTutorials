% MAIN.m  --  Lesson 3 -- Multiple Drawing
%
% This script performs a simulation of the cart as it passively moves from
% some initial state. The system is drawn in a sub-plot for nine uniformly
% spaced states throughout the simulation.
%
% In this lesson I've pushed the drawing commands into a function so that
% we can call them multiple times from a for loop. I've also introduced the
% concept of interpolating the data to get the position of the system at
% the desired times.
%

clc; clear;

%%%% Initial State
z0 = [
    0.0;   %horizontal position
    (pi/180)*80;  %pendulum angle (wrt gravity)
    0.3;   %horizontal velocity
    0.0];  %pendulum angular rate

%%%% Physical Parameters
p.m1 = 1.0;  % (kg) Cart mass
p.m2 = 0.3;  % (kg) pole mass
p.g = 9.81;  % (m/s^2) gravity 
p.l = 0.5;   % (m) pendulum (pole) length 

%%%% Time vector
t = linspace(0,2,250);  %Simulation time stamps

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


%%%% Drawing:

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

% Create and clear a figure:
figure(2); clf; 

% Plot the cart-pole at several different times:
timeDraw = linspace(t(1),t(end),9);
posDraw = interp1(t',pos',timeDraw')';
idx = 0;
for i=1:3
    for j=1:3
        idx = idx + 1;
        subplot(3,3,idx); hold on;
        drawCartPole(timeDraw(idx),posDraw(:,idx),extents);
    end
end




