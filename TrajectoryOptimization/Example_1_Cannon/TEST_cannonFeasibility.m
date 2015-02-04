% TEST_cannonDynamics.m
%
% This script runs a simple simulation to demonstrate the dynamics of the
% cannon ball problem.
%

clc; clear;

% Cannon ball starts at the origin
x0 = 0;
y0 = 0;

% Prescribe initial conditions with velocity and angle
nSim = 20;
v0 = logspace(-2,6,nSim);
th0 = 45*(pi/180);

% Quadratic drag coefficient:
c = 0.5;

% Store the range as a function of initial angle:
xRange = zeros(1,nSim);

for iSim=1:nSim

% Set up initial conditions for ode45
dx0 = v0(iSim)*cos(th0);
dy0 = v0(iSim)*sin(th0);

% Set up arguments for ode45
userFun = @(t,z)cannonDynamics(t,z,c);  %Dynamics function
tSpan = [0,100];  %Never plan on reaching final time
z0 = [x0;y0;dx0;dy0];  %Initial condition
options = odeset('Events',@groundEvent,'Vectorized','on','AbsTol',1e-8, 'RelTol',1e-8);

% Run a simulation
sol = ode45(userFun, tSpan, z0, options);

xRange(iSim) = sol.y(1,end);

fprintf('Initial Speed: %6.6g,  Horizontal Range: %6.6g \n',v0(iSim),xRange(iSim));

end

% Plot the range as a function of the initial speed
plot(v0,xRange,'kx','LineWidth',2,'MarkerSize',10)
set(gca, 'Xscale', 'log')
set(gca, 'Yscale', 'log')
xlabel('Initial Speed')
ylabel('Horizontal Range');
title('Range vs Initial Speed');

