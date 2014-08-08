%TEST_SmoothAbsFancy
%
% This function plots the output of SmoothAbsFancy for a few different
% values of alpha (smoothin parameter) on the domain [-1,1]. See
% SmoothAbsFancy for more details.
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

t = linspace(-1,1,1000);

slope1 = 0.5;   
slope2 = 1;

alpha = [0.001,0.01,0.1];
N=length(alpha);

figure(103); clf; hold on;
for i=1:N
    x1 = SmoothAbsFancy(t,alpha(i),slope1,slope2);

    subplot(N,1,i); hold on;
    plot(t,x1,'b-','LineWidth',2)
    plot(t(t<0),-slope1*t(t<0),'k:')
    plot(t(t>=0),slope2*t(t>=0),'k:')
    title(['Alpha = ' num2str(alpha(i))],'FontSize',14);
end