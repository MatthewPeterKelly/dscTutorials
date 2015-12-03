% MAIN.m  --  Lesson 4 -- Simple Animation
%
% This script performs an simulation and animation of a cart-pole as it
% moves passively along a horizontal track.
%
% In this lesson we've moved to a real-time animation. The computer plots
% as fast as it can, then checks how much time has passed, and then plots
% that state at that new time. This method for plotting will keep track
% with "real time".

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

% Create and clear a figure:
figure(2); clf; 

time = 0;
tic;
while time < t(end)
    
    % Compute the position of the system at the current real world time
    time = toc;
    posDraw = interp1(t',pos',time')';
    
    % Clear the figure and plot the system in its current stae
    clf; hold on;
    drawCartPole(time,posDraw,extents);
    
    % Push the draw commands through the plot buffer
    drawnow;
    
end

% Show the final state once animation is done:
drawCartPole(t(end),pos(:,end),extents);
