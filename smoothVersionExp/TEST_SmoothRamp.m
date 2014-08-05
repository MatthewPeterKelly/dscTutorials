%TEST_SmoothRamp
%
% This script plots the output of SmoothRamp for a few values of alpha, on
% the domain [-1,1]
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

t = linspace(-1,1,1000);
alpha = [0.02,0.05,0.2];

figure(101); clf; hold on;
for i=1:length(alpha)
    x1 = SmoothRamp(t,alpha(i));

    subplot(N,1,i); hold on;
    plot(t,x1,'LineWidth',2)
    plot(t(t<0),0,'k:')
    plot(t(t>=0),t(t>=0),'k:')
    title(['Alpha = ' num2str(alpha(i))],'FontSize',14);
end