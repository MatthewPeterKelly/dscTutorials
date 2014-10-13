%MAIN_doublePendulum.m
%
% This script runs a simulation of a double pendulum

% Use: EoM_Double_Pendulum to write the equations of motion

%Physical parameters:
m1 = 1.0;  % (kg) pendulum mass, link 1
m2 = 1.0;  % (kg) pendulum mass, link 2
g = 9.81; % (m/s^2) gravity
l1 = 1.0; % (m) link 1 length
l2 = 1.0; % (m) link 2 length
d1 = 0.5*l1; % Distance from parent joint to link CoM
d2 = 0.5*l2; % Distance from parent joint to link CoM
I1 = m1*l1*l1/12; %Link 1, moment of inertia
I2 = m2*l2*l2/12; %Link 2, moment of inertia

tSpan = [0,10]; %Simulation time interval

z0(1) = (pi/180)*(0);  % Link 1, initial abs angle
z0(2) = 0;  % Link 1, initial abs angle rate
z0(3) = (pi/180)*(0);  % Link 1, initial abs angle
z0(4) = 0;  % Link 1, initial abs angle rate

userFunc = @(t,z)doublePendulumRhs(t,z,m1,m2,g,l1,I1,I2,d1,d2);

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
[energy, kinetic, potential] = doublePendulumEnergy(th1,th2,w1,w2,...
    m1,m2,g,l1,l2,I1,I2,d1,d2);

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
doublePendulumAnimate(sol,l1,l2);

