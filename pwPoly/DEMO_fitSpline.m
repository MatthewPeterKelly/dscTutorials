% DEMO_fitSpline.m
%
% Fit a spline to data, which can then be evaluated by ppval


tSpan = [0,10];
nData = 100;
nKnot = 10;

tData = linspace(tSpan(1),tSpan(2),nData);
xData = sin(tData);
yData = cos(tData);

tKnot = linspace(tSpan(1),tSpan(2),nKnot);

pp = fitSpline(tData, [xData;yData], tKnot);

zSpline = ppval(pp,tData);

figure(15); clf; 

subplot(2,1,1); hold on;
plot(tData,xData,'ko')
plot(tData,zSpline(1,:),'b-')
legend('data','spline');

subplot(2,1,2); hold on;
plot(tData,yData,'ko')
plot(tData,zSpline(2,:),'b-')
legend('data','spline');