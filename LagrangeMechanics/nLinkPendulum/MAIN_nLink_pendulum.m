%MAIN_doublePendulum.m
%
% This script runs a simulation of a nLink pendulum

clear; clc;

N = 10; 
overwrite = false;
EoM_nLink_pendulum(N,overwrite);

%Physical parameters:
P.g = 1.0;
P.m = 1.0*ones(N,1) + 1.5*rand(N,1);
P.l = 1.0*ones(N,1) + 0.4*rand(N,1);
P.I = (1/12)*P.m.*P.l.^2;
P.d = P.l.*(0.25 + 0.5*rand(N,1));

tSpan = [0,4]; %Simulation time interval

th0 = (pi/180)*(5 + 15*rand(N,1));
dth0 = zeros(N,1);
z0 = [th0;dth0];

userFunc = eval(['@(t,z)dynamics_' num2str(N) '_link(t,z,P);']);
% % userFunc = eval(['@(t,z)dynamics_' num2str(N) '_link_mex(t,z,P);']);

options = odeset(...
    'AbsTol',1e-6,...
    'RelTol',1e-6,...
    'Vectorized','on');

% Run the simulation!
tic
sol = ode45(userFunc,tSpan,z0,options);
toc


% Break apart solution for plotting
nPlot = 1000;
time = linspace(tSpan(1),tSpan(2),nPlot);
z = deval(sol,time); %Evaluate solution from ode45 at points in time
th = z(1:N,:);
w = z(N+1:2*N,:);
[energy, kinetic, potential] = eval(['energy_' num2str(N) '_link(z,P);']);

% Plotting!

figure(2222);clf;

subplot(3,1,1); hold on;
plot(time,th,'LineWidth',2);
xlabel('time (s)')
ylabel('angle (rad)');

subplot(3,1,2); hold on;
plot(time,w,'LineWidth',2);
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
nLinkAnimate(sol,P);

