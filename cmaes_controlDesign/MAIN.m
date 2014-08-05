%MAIN - This script runs cmaes to solve for the coefficients of a
%controller for a nonlinear pendulum controller.

clear; clc;

%Controller is designed using nSimulations parallel simulations
nSimulations = 100;
IC = [1.2*(0.5-rand(1,nSimulations));  %Random initial angle
    0.7*(0.5-rand(1,nSimulations))];  %Random initial rate

%The controller is tested using a different set of initial states
nTest = 10;
Test_IC = [1.2*(0.5-rand(1,nTest));  %Random initial angle
    0.7*(0.5-rand(1,nTest))];  %Random initial rate

fitfun = 'objective';
xstart = [0;0;0];   %Initial guess at the parameters
insigma = [4;1;10];  %Search variance for the paramters
inopts = cmaes();
inopts.DispModulo = 40;
inopts.SaveVariables = 'off';


[xmin, ...      % minimum search point of last iteration
    fmin, ...      % function value of xmin
    counteval, ... % number of function evaluations done
    stopflag, ...  % stop criterion reached
    out, ...     % struct with various histories and solutions
    bestever ... % struct containing overall best solution (for convenience)
    ] = cmaes( ...
    fitfun, ...    % name of objective/fitness function
    xstart, ...    % objective variables initial point, determines N
    insigma, ...   % initial coordinate wise standard deviation(s)
    inopts,...
    IC);    % options struct, see defopts below

%Plot the solution:
figNum = 1;
plotcmaesdat(figNum);

%Remove the data files that cmaes creates at runtime
delete outcmaes*.dat;

%Test Simulation
k = bestever.x;
figure(2); clf; hold on;
subplot(3,1,1); hold on;
subplot(3,1,2); hold on;
subplot(3,1,3); hold on;
for testNum = 1:nTest
    [t, z, u] = simulate(k,Test_IC(:,testNum));
    subplot(3,1,1);
    plot(t,z(1,:));
    subplot(3,1,2);
    plot(t,z(2,:));
    subplot(3,1,3);
    plot(t,u);
end

%Add title and axis names
subplot(3,1,1);
xlabel('Time (s)');
ylabel('Angle (rad)');
title(...
    ['Controller = $' num2str(k(1)) ' \theta + ' num2str(k(2)) ' \dot{\theta} + ' num2str(k(3)) '\sin(\theta)$'],...
    'interpreter','latex',...
    'FontSize',16);
subplot(3,1,2);
xlabel('Time (s)');
ylabel('Rate (rad/s)');
subplot(3,1,3);
xlabel('Time (s)');
ylabel('Torque (Nm)');
