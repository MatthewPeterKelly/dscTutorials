%plotData.m
%
% This script loads the data file produced by the c++ code and then plots
% it to a figure.
%

data = csvread('data.csv');
t = data(:,1);
p = data(:,2);
v = data(:,3);

figure(1); clf;
subplot(2,1,1)
plot(t,p)
title('position')
subplot(2,1,2)
plot(t,v)
title('velocity')