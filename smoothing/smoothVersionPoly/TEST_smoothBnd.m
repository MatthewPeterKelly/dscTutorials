%TEST_smoothBnd

x = linspace(-3,4,1000);
Bnd = [-1,2];

alpha = [0.2,0.5,1.5];
order = [1,3,8];

N = length(alpha);
M = length(order);

figure(113); clf;
index = 0;
for i=1:N
    for j=1:M
        index=index+1;
        subplot(N,M,index); hold on;
        y = smoothBnd(x,Bnd,alpha(i),order(j));
        plot(x,y,'b-','LineWidth',2)
            c1 = x<Bnd(1);
            c3 = x>=Bnd(2);
            c2 = (~c1)&(~c3);
        plot(x(c1),Bnd(1)*ones(size(x(c1))),'k--','LineWidth',1);
        plot(x(c2),x(c2),'k--','LineWidth',1);
        plot(x(c3),Bnd(2)*ones(size(x(c3))),'k--','LineWidth',1);
        title(['alpha = ' num2str(alpha(i)) ',  order = ' num2str(order(j))]...
            ,'FontSize',14);
    end
end

