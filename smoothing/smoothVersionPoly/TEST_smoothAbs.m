%TEST_smoothAbs

x = linspace(-1,1,1000);

alpha = [0.05,0.2,0.8];
order = [1,3,8];

N = length(alpha);
M = length(order);

figure(111); clf;
index = 0;
for i=1:N
    for j=1:M
        index=index+1;
        subplot(N,M,index); hold on;
        y = smoothAbs(x,alpha(i),order(j));
        plot(x,y,'b-','LineWidth',2)
        plot(x,abs(x),'k--','LineWidth',1);
        title(['alpha = ' num2str(alpha(i)) ',  order = ' num2str(order(j))]...
            ,'FontSize',14);
    end
end


