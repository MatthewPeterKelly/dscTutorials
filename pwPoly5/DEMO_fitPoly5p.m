% DEMO_fitPoly5.m
%
% Demonstrates useing pwPoly5 to approximate a periodic function
%

clc; clear;

%How many knot points to use on the curve
nKnot = 5;  

% Build up a test function: (polar coordinates)
t = linspace(-pi,pi,200);
T = linspace(-pi,pi,nKnot);
r = 2+sin(2*t)+0.6*cos(4*t+2.05);
r = r + 0.05*randn(size(t));  % Add some noise:

% Fit the piecewise-cubic function
P = fitPoly5p(T,t,r);

% Evaluate the fit:
rFit = pwPoly5(T,P,t);

% Convert to cartesian for plotting:
[x,y] = pol2cart(t,r);
[xFit,yFit] = pol2cart(t,rFit);
[xKnot,yKnot] = pol2cart(T,P(1,:));

% Plot the solution against data set:
figure(5); clf; hold on;
plot(xFit,yFit,'r-')
plot(x,y,'k.')
plot(xKnot,yKnot,'bo','MarkerSize',15)
xlabel('x')
ylabel('y')
title('periodic test function')