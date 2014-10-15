%MAIN_doublePendulum.m
%
% This script runs a simulation of a double pendulum

% Use: EoM_Double_Pendulum to write the equations of motion

clear; clc;

%Physical parameters:
P.m1 = 4.0;  % (kg) pendulum mass, link 1
P.m2 = 1.0;  % (kg) pendulum mass, link 2
P.g = 9.81; % (m/s^2) gravity
P.l1 = 1.0; % (m) link 1 length
P.l2 = 2.0; % (m) link 2 length
P.d1 = 0.6*P.l1; % Distance from parent joint to link CoM
P.d2 = 0.4*P.l2; % Distance from parent joint to link CoM
P.I1 = P.m1*P.l1^2/12; %Link 1, moment of inertia
P.I2 = P.m2*P.l2^2/12; %Link 2, moment of inertia

tSpan = [0,10]; %Simulation time interval

z0 = zeros(4,1);
z0(1) = (pi/180)*(0);  % Link 1, initial abs angle
z0(2) = 0;  % Link 1, initial abs angle rate
z0(3) = (pi/180)*(0);  % Link 1, initial abs angle
z0(4) = 0;  % Link 1, initial abs angle rate

userFunc = @(t,z)doublePendulumDynamics(t,z,P);

options = odeset(...
    'AbsTol',1e-6,...
    'RelTol',1e-6,...
    'Vectorized','on');

% Run the simulation!
sol = ode45(userFunc,tSpan,z0,options);

% Break apart solution for plotting
nPlot = 1000;
time = linspace(tSpan(1),tSpan(2),nPlot);
z = deval(sol,time); %Evaluate solution from ode45 at points in time
th1 = z(1,:);
w1 = z(2,:);
th2 = z(3,:);
w2 = z(4,:);
[energy, kinetic, potential] = doublePendulumEnergy(z,P);

% Plotting!

figure(2222);clf;

subplot(3,1,1); hold on;
plot(time,th1,'r-','LineWidth',2);
plot(time,th2,'b-','LineWidth',2);
xlabel('time (s)')
ylabel('angle (rad)');

subplot(3,1,2); hold on;
plot(time,w1,'r-','LineWidth',2);
plot(time,w2,'b-','LineWidth',2);
xlabel('time (s)')
ylabel('angle rate (rad/s)');

subplot(3,1,3);  hold on;
datum = min(potential);
plot(time,energy - datum,'k-','LineWidth',3);
plot(time,kinetic,'LineWidth',2,'color',[0.5,0.1,0.6]);
plot(time,potential - datum,'LineWidth',2,'color',[0.3,0.5,0.2]);
xlabel('time (s)')
ylabel('energy (J)');
legend('total','kinetic','potential');

% Animation
figure(3333); clf;
doublePendulumAnimate(sol,P);

