%TEST_smoothRamp

x = linspace(-1,1,1000);

alpha = [0.05,0.2,0.8];
order = [1,3,8];

N = length(alpha);
M = length(order);

figure(112); clf;
index = 0;
for i=1:N
    for j=1:M
        index=index+1;
        subplot(N,M,index); hold on;
        y = smoothRamp(x,alpha(i),order(j));
        plot(x,y,'b-','LineWidth',2)
        plot(x(x<0),0*x(x<0),'k--','LineWidth',1);
        plot(x(x>=0),x(x>=0),'k--','LineWidth',1);
        title(['alpha = ' num2str(alpha(i)) ',  order = ' num2str(order(j))]...
            ,'FontSize',14);
    end
end


