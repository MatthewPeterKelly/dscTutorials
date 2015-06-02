% TEST_cartPoleDynamics.m
%
% A simple script to run a forward simulation of the cart-pole dynamics, to
% make sure that everything looks reasonable.
clc; clear;

p.m1 = 1;
p.m2 = 1;
p.g = 9.81;
p.l = 1;

z0 = [...
    0;              %Initial horizontal position
    (pi/180)*120;   %Initial angle of the pendulum (from stable config)
    0;              %Initial velocity of cart
    0];             %initial rotation rate of pendulum

u = [0;0];  %Disable actuators for now;

tSpan = [0;4];   %Time span for simulation

dynFun = @(t,z)( cartPoleDynamics(z,u,p) );   %Function handle to give ode45
sol = ode45(dynFun,tSpan,z0);  %Run simulation

%Extract uniformly spaced samples via high-order interpolation
t = linspace(tSpan(1),tSpan(2),250);
z = deval(sol,t);  

% Make some simple plots:
figure(1); clf; 

subplot(2,2,1);
plot(t,z(1,:));
xlabel('t')
ylabel('x')

subplot(2,2,2);
plot(t,z(2,:));
xlabel('t')
ylabel('q')

subplot(2,2,3);
plot(t,z(3,:));
xlabel('t')
ylabel('dx')

subplot(2,2,4);
plot(t,z(4,:));
xlabel('t')
ylabel('dq')

