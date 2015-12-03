% MAIN.m  --  Lesson 2 -- Simple Drawing
%
% This script performs a simulation of the cart as it passively moves from
% some initial state.
%
% In the lesson I've moved the state plotting commands to their own
% function, and introduced some basic drawing concepts using plot
%

clc; clear;

%%%% Initial State
z0 = [
    0.0;   %horizontal position
    (pi/180)*120;  %pendulum angle (wrt gravity)
    0.0;   %horizontal velocity
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
plotPendulumCart(t,z);  %Moved plotting to its own function


%%%% Drawing:

% Convert states to cartesian positions:
pos = cartPolePosition(z,p); 

% Unpack the positions:
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

% Select an index corresponding to the time step to draw:
idx = 6;  %Draw the initial state for now

% Create and clear a figure:
figure(2); clf; 
hold on;  %   <-- Important!

title('Cart-Pole System')

% Draw the rail that the cart-pole travels on
plot([xLow, xUpp],[0,0],'k-','LineWidth',2);

% Draw the cart:
plot(x1(idx), y1(idx), 'bs','MarkerSize',30,'LineWidth',5);

% Draw the pole:
plot([x1(idx),x2(idx)], [y1(idx), y2(idx)], 'r-','LineWidth',2);

% Draw the bob of the pendulum:
plot(x2(idx), y2(idx), 'ro','MarkerSize',22,'LineWidth',4);

% Format the axis so things look right:
axis(extents); axis equal; axis off;




