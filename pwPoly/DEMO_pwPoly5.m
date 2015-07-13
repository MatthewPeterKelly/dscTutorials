% DEMO_smoothCurve.m
%
% This script demonstrates how to use smoothCurve to create a piecewise
% smooth curve that is twice differentiable
%

% The number of knots is the number of segments + 1
nKnot = 5;

% P = function [value; slope; curvature] at knot points
P = [...
    2*(1-2*rand(1,nKnot));  %Fucntion value
    1*randn(1,nKnot);   %Function slope
    5*randn(1,nKnot)];  %Function curvature

% T is the location of each knot point:
T = cumsum(0.8+0.2*rand(1,nKnot));

% Evaluate the piecewise function:
t = linspace(T(1),T(end),250);
[x,dx,ddx] = pwPoly5(T,P,t);

% Plot the results:
figure(87); clf;
subplot(3,1,1); hold on;
plot(t,x,'k-');
plot(T,P(1,:),'bo','MarkerSize',10)
ylabel('x')
title('Test Function')
subplot(3,1,2); hold on;
plot(t,dx,'k-');
plot(T,P(2,:),'bo','MarkerSize',10)
ylabel('dx')
subplot(3,1,3); hold on;
plot(t,ddx,'k-');
plot(T,P(3,:),'bo','MarkerSize',10)
ylabel('ddx')
xlabel('t')


