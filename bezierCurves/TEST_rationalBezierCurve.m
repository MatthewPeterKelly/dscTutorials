% TEST_rationalBezierCurve.m
%
% This script runs a simple demo of a rational bezier curve

order = 5;

p = rand(2,order+1);
w = 0.1+rand(1,order+1);   %Must be positive for convex hull property to hold!

tSpan = [1,4];
t = linspace(tSpan(1),tSpan(2),100);

tic
x = rationalBezierCurve(p,w,t,tSpan);
toc

figure(1); clf;
plot(x(1,:),x(2,:)); hold on;
plot(p(1,:),p(2,:),'x');