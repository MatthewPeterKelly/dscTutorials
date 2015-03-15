%TEST_bezierFunction.m
%
% This script tests the use of a bezier curve to represent a vector
% function, rather than an arbitrary space curve
%

order = 4;

p = rand(1,order+1);


tSpan = [1,4];
t = linspace(tSpan(1),tSpan(2),100);
pGrid = linspace(tSpan(1),tSpan(2),order+1);

tic
x = bezierCurve(p,t,tSpan);
toc

figure(1); clf;
plot(t,x); hold on;
plot(pGrid,p,'x');