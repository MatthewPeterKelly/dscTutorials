%TEST_SmoothBnd
%
% This function plots the output of SmoothBnd for a few sample values of
% the smoothing parameter alphaover the domain [-1,1]
%
% Written by Matthew Kelly
% October 2013
% Cornell University
%

t = linspace(-1,2,1000);
Bnd = [0,1];
alpha = [0.01,0.05,0.2];
N=length(alpha);


figure(102); clf; hold on;
for i=1:N
    x1 = SmoothBnd(t,alpha(i),[0,1]);

    subplot(N,1,i); hold on;
    plot(t,x1,'b-','LineWidth',2)
    c1 = t<Bnd(1); c2 = t>=Bnd(2); c3 = ~c1&~c2;
    plot(t(c1),Bnd(1),'k:')
    plot(t(c2),Bnd(2),'k:')
    plot(t(c3),t(c3),'k:')
    title(['Alpha = ' num2str(alpha(i))],'FontSize',14);
end