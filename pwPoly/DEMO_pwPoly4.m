% DEMO_pwPoly4.m
%
% Demonstrate usage of pwPoly4 for quadratic interpolation

clc; clear;

tGrid = linspace(0,7,15);
xGrid = sin(tGrid);
dxGrid = cos(tGrid);

t = linspace(tGrid(1),tGrid(end),200);
x = pwPoly4(tGrid,xGrid,dxGrid,t);


figure(2); clf; hold on;
plot(t,sin(t),'o')
plot(t,x)
legend('interp','analytic');