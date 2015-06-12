% MAIN_passiveSimulate.m
%
% This script runs a passive simulation of the acrobot, to do a sanity
% check on the dynamics and plotting routines
%
clc; clear;

p.m1 = 1;
p.m2 = 1;
p.g = 9.81;
p.l1 = 1;
p.l2 = 1;

% Initial state:
q1 = (pi/180)*120;
q2 = (pi/180)*170;
dq1 = 0;
dq2 = 0;
z0 = [q1;q2;dq1;dq2];  %Pack up initial state

tSpan = [0,2];  %time span for the simulation
dynFun = @(t,z)( acrobotDynamics(z,0,p) );  %passive dynamics function

% Run simulation:
sol = ode45(dynFun,tSpan,z0);
t = linspace(tSpan(1),tSpan(2),150);
z = deval(sol,t);
u = zeros(size(t));

% Plot the results:
figure(1337); clf; plotAcrobot(t,z,u,p)
