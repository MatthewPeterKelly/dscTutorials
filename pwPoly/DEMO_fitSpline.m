% DEMO_fitSpline.m
%
% Fit a spline to data, which can then be evaluated by ppval


tSpan = [0,10];
nData = 100;
nKnot = 10;

tData = linspace(tSpan(1),tSpan(2),nData);
xData = sin(tData);

tKnot = linspace(tSpan(1),tSpan(2),nKnot);

pp = fitSpline(tData, xData, tKnot);

xSpline = ppval(pp,tData);

figure(15); clf; hold on;
plot(tData,xData,'ko')
plot(tData,xSpline,'b-')
legend('data','spline');