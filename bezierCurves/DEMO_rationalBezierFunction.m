%TEST_bezierFunction.m
%
% This script tests the use of a bezier curve to represent a vector
% function, rather than an arbitrary space curve
%

order = 2;

p = rand(1,order+1);
w = 0.05 + 0.95*rand(1,order+1);

tSpan = [1,4];
t = linspace(tSpan(1),tSpan(2),100);
pGrid = linspace(tSpan(1),tSpan(2),order+1);

tic
x = rationalBezierCurve(p,w,t,tSpan);
toc

figure(1); clf;
plot(t,x); hold on;
scatter(pGrid,p,50*w,'filled');