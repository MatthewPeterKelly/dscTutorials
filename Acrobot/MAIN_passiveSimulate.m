% MAIN_passiveSimulate.m
%
% This script runs a passive simulation of the acrobot, to do a sanity
% check on the dynamics and plotting routines
%
% Things to try:
%
% 1) Adjust the parameters and states and see if you can guess what will
% happen when you run the simulation. What if one mass or length is much
% bigger than the other? Can you make the system behave like a single 
% pendulum?
% 
% 2) Type:      >> help animate 
% to see the keyboard commands for controlling the animation. You can use
% then to see the system in slow motion, pause, or go back in time, to name
% a few possiblities. This is useful for understanding what the system is
% doing.
%

clc; clear;

% Physical parameters
p.m1 = 10;
p.m2 = 20;
p.g = 9; 
p.l1 = 3;
p.l2 = 5;
 
% Initial state:
q1 = (pi/180)*120;
q2 = (pi/180)*50;
dq1 = 0;
dq2 = 0;
z0 = [q1;q2;dq1;dq2];  %Pack up initial state

tSpan = [0,8];  %time span for the simulation
dynFun = @(t,z)( acrobotDynamics(z,0,p) );  %passive dynamics function

% Run simulation:
sol = ode45(dynFun,tSpan,z0);
t = linspace(tSpan(1),tSpan(2),500);
z = deval(sol,t);
u = zeros(size(t));

% Animate the results:
A.plotFunc = @(t,z)( drawAcrobot(t,z,p) );
A.speed = 1.0;
A.figNum = 101;
A.verbose = true;
animate(t,z,A)

% Plot the results:
figure(1337); clf; plotAcrobot(t,z,u,p)
