% MAIN_passiveSimulate.m
%
% This script runs a passive simulation of the acrobot, to get a rough
% trajectory that is similar to a single walking step. 
%

clc; clear;

p.m1 = 10;
p.m2 = 1;
p.g = 5;
p.l1 = 2;
p.l2 = 2;

% Initial state:
th0 = (pi/180)*30;
dth0 = (pi/180)*150;
q1 = (pi/180)*180 + th0;
q2 = -th0;
dq1 = -dth0;
dq2 = dth0;
z0 = [q1;q2;dq1;dq2];  %Pack up initial state

tSpan = [0,1];  %time span for the simulation
dynFun = @(t,z)( acrobotDynamics(z,0,p) );  %passive dynamics function

% Run simulation:
sol = ode45(dynFun,tSpan,z0);
t = linspace(tSpan(1),tSpan(2),500);
z = deval(sol,t);
u = zeros(size(t));

% Animate the results:
A.plotFunc = @(t,z)( drawAcrobot(t,z,p) );
A.speed = 0.2;
A.figNum = 101;
animate(t,z,A)

% Plot the results:
figure(1337); clf; plotAcrobot(t,z,u,p)
