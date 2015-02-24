% TEST_cannonDynamics.m
%
% This script runs a simple simulation to demonstrate the dynamics of the
% cannon ball problem.
%

clc; clear;

% Cannon ball starts at the origin
x0 = 0;
y0 = 0;

% Prescribe initial conditions with velocity and angle
v0 = 10;
th0 = 45*(pi/180);

% Quadratic drag coefficient:
c = 0.2;

% Set up initial conditions for ode45
dx0 = v0*cos(th0);
dy0 = v0*sin(th0);

if dy0 < 0, error('Cannot point cannon through ground! sin(th0) > 0 Required.'); end;

% Set up arguments for ode45
userFun = @(t,z)cannonDynamics(t,z,c);  %Dynamics function
tSpan = [0,100];  %Never plan on reaching final time
z0 = [x0;y0;dx0;dy0];  %Initial condition
options = odeset('Events',@groundEvent,'Vectorized','on');

% Run a simulation
sol = ode45(userFun, tSpan, z0, options);

% Extract the solution on uniform grid:
t = linspace(sol.x(1), sol.x(end), 100);
z = deval(sol,t);
x = z(1,:); xGround = [x(1)-0.1*range(x), x(end)+0.1*range(x)]; 
y = z(2,:); yGround = [0, 0];

% Plot the solution:
figure(100); clf; hold on;
plot(xGround, yGround,'color',[0.5, 0.2, 0.1],'LineWidth',4);
drawTree(x(1) + 0.6*range(x), 0, 1);  %Plot a tree for scale
plot(x(1),y(1),'b.','MarkerSize',35);   %Start
plot(x(end),y(end),'rx','LineWidth',4,'MarkerSize',14)   %Finish
plot(x,y,'k-','LineWidth',3);  %Trajectory
axis equal;
xlabel('Horizontal Position')
ylabel('Vertical Position')
title('Cannon Ball Trajectory')

