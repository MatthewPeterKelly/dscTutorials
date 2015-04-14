% TEST_bezierCurve.m
%
% This script runs a simple demo of a bezier curve

order = 5;

p = rand(2,order+1);

tSpan = [1,4];
t = linspace(tSpan(1),tSpan(2),100);

tic
x = bezierCurve(p,t,tSpan);
toc

figure(1); clf;
plot(x(1,:),x(2,:)); hold on;
plot(p(1,:),p(2,:),'x');
