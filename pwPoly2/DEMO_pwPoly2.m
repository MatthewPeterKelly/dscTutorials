% DEMO_pwPoly2.m
%
% Demonstrate usage of pwPoly2 for quadratic interpolation

clc; clear;

tGrid = linspace(0,7,15);
xGrid = [sin(tGrid); cos(tGrid)];

t = linspace(tGrid(1),tGrid(end),200);
x = pwPoly2(tGrid,xGrid,t);


figure(1); clf; 
subplot(2,1,1); hold on;
plot(t,sin(t),'o')
plot(t,x(1,:))
legend('interp','analytic');
subplot(2,1,2); hold on;
plot(t,cos(t),'o')
plot(t,x(2,:))
legend('interp','analytic');