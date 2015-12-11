%TEST_SmoothAbs
%
% This script plots the output of SmoothAbs for a few values of alpha, over
% the domain [-1,1]
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

t = linspace(-1,1,1000);
alpha = [0.02,0.1,0.5];
N=length(alpha);

figure(104); clf; hold on;
for i=1:N
    x1 = SmoothAbs(t,alpha(i));

    subplot(N,1,i); hold on;
    plot(t,x1,'b-','LineWidth',2)
    plot(t,abs(t),'k:')
    title(['Alpha = ' num2str(alpha(i))],'FontSize',14);
end