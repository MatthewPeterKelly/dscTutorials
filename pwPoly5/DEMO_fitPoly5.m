% DEMO_fitPoly5.m
%
% Demonstrates useing pwPoly5 to approximate a data set
%

clc; clear;

nKnot = 6;  %Number of knot points for piecewise-polynomial

% Create a test data set
t = linspace(-pi,pi,100);
T = linspace(-pi,pi,nKnot);
x = sin(1.5*t) + 1.8*exp(-1.5*t.^2);
x = x + 0.04*randn(size(t)); % Add some noise:

% Find the best-fit polynomial
P = fitPoly5(T,t,x);

% Evaluate the fit:
xFit = pwPoly5(T,P,t);

% Plot the solution:
figure(5); clf; hold on;
plot(t,xFit,'r-')
plot(t,x,'k.')
plot(T,P(1,:),'bo','MarkerSize',15)
xlabel('t')
ylabel('x')
title('test function')