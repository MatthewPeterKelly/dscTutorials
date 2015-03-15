%TEST_fitBezierCurve.m
%
% This script runs a test to make sure that the bezier curve fitting
% function is working properly.
%

nData = 25;
tSpan = [-1,2];
t = linspace(tSpan(1),tSpan(2),nData);
x = 2*sin(2*t) + cos(3*t+1);
y = 1-exp(-2*t).*cos(2*t);
z = [x;y];

nCtrlPt = 4;
xBnd = [-3,3];
yBnd = [-5,5];
zBnd = [xBnd;yBnd];

[p,w] = fitBezierCurve(t,z,nCtrlPt,tSpan,zBnd);  

tFit = linspace(tSpan(1),tSpan(2),100);
zFit = rationalBezierCurve(p,w,tFit,tSpan);

figure(2); clf;
subplot(2,1,1);
plot(t,z(1,:),'x',tFit,zFit(1,:));
legend('data','fit');
xlabel('time');
ylabel('function one');
subplot(2,1,2);
plot(t,z(2,:),'x',tFit,zFit(2,:));
legend('data','fit');
xlabel('time');
ylabel('function two');