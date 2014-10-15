%MAIN_singlePendulum.m
%
% This script runs a simple simulation of a single pendulum

% Use: EoM_Single_Pendulum to write the equations of motion

m = 1.0;  % (kg) pendulum mass
g = 9.81; % (m/s^2) gravity
l = 1.0; % (m) pendulum length

tSpan = [0,10]; %Simulation time interval

th0 = (pi/180)*(-178);  %Initial angle
w0 = 0;  %Initial angular rate
z0 = [th0;w0];

userFunc = @(t,z)singlePendulumRhs(t,z,g,l);

options = odeset(...
    'AbsTol',1e-8,...
    'RelTol',1e-8,...
    'Vectorized','on');

% Run the simulation!
sol = ode45(userFunc,tSpan,z0,options);

% Break apart solution for plotting
nPlot = 1000;
time = linspace(tSpan(1),tSpan(2),nPlot);
z = deval(sol,time); %Evaluate solution from ode45 at points in time
th = z(1,:);
w = z(2,:);
[energy, kinetic, potential] = singlePendulumEnergy(th,w,m,g,l);

% Plotting!

figure(1111);clf;

subplot(3,1,1);
plot(time,th,'k-','LineWidth',2);
xlabel('time (s)')
ylabel('angle (rad)');

subplot(3,1,2);
plot(time,w,'k-','LineWidth',2);
xlabel('time (s)')
ylabel('angle rate (rad/s)');


subplot(3,1,3);  hold on;
plot(time,energy,'k-','LineWidth',3);
plot(time,kinetic,'r-','LineWidth',2);
plot(time,potential,'b-','LineWidth',2);
xlabel('time (s)')
ylabel('energy (J)');
legend('total','kinetic','potential');


