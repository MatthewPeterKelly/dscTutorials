% MAIN.m  --  Lesson 1 -- Simple Plot
%
% This script performs a simulation of the cart as it passively moves from
% some initial state.
%
% The purpose of this lesson is to learn how use ode45 to generate data for
% a simulation, and then plot the state of the system to see what is going
% on.
%

clc; clear;

%%%% Initial State
z0 = [
    0.0;   %horizontal position
    (pi/180)*80;  %pendulum angle (wrt gravity)
    0.3;   %horizontal velocity
    0.5];  %pendulum angular rate

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

%%%% Unpack the state:
x = z(1,:);
q = z(2,:);
dx = z(3,:);
dq = z(4,:);

%%%% Plots:
figure(1); clf;

subplot(2,2,1);
plot(t,x)
ylabel('x')
title('Position')

subplot(2,2,2);
plot(t,q)
ylabel('q')
title('Angle')

subplot(2,2,3);
plot(t,dx)
ylabel('dx')
title('Velocity')

subplot(2,2,4);
plot(t,dq)
ylabel('dq')
title('Angle Rate')




