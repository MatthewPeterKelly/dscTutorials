%MAIN_simpleHarmonicOscillator.m
%
% This script runs a simulation of a simple harmonic oscillator

% Use: EoM_Single_Pendulum to write the equations of motion

m = 1.0;  % (kg)  mass
k = 1.0;  % (N/m) spring constant

tSpan = [0,10]; %Simulation time interval

x0 = 0.5;  % (m) initial position 
v0 = 0;  % (m/s) initial velocit
z0 = [x0;v0];

userFunc = @(t,z)simpleHarmonicOscillatorDynamics(t,z,m,k);

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
x = z(1,:);
v = z(2,:);


% Plotting!

figure(111);clf;

subplot(2,1,1);
plot(time,x,'k-','LineWidth',2);
xlabel('time (s)')
ylabel('position (m)');

subplot(2,1,2);
plot(time,v,'k-','LineWidth',2);
xlabel('time (s)')
ylabel('velocity (m/s)');


