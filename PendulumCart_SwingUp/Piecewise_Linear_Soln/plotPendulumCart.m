function plotPendulumCart(t,x,param)

Cart_Width = 0.2;
Cart_Height = 0.1;
Limits = param.axis;

Cart = x(1:2);
Bob = x(3:4);

if param.clearFigure    %THEN RUNNING ANIMATION
    clf;
    hold on;
    
    %Plot Cart
    x = Cart(1) - 0.5*Cart_Width;
    y = -0.5*Cart_Height;
    w = Cart_Width;
    h = Cart_Height;
    h = rectangle('Position',[x,y,w,h],'LineWidth',4,'Curvature',[0.3,0.3]);
    set(h,'EdgeColor',[0.1,0.8,0.1])
    
    %Plot Pendulum
    Rod_X = [Cart(1), Bob(1)];
    Rod_Y = [Cart(2), Bob(2)];
    plot(Rod_X,Rod_Y,'k-','LineWidth',4)
    
    %Plot Bob
    plot(Bob(1),Bob(2),'k.','MarkerSize',50)
    
    %Plot Rails
    plot([Limits(1) Limits(2)],-0.5*Cart_Height*[1,1],'k-','LineWidth',2)
    
    %Title
    title(['Simulation Time: ' num2str(t) ' s'])
    
    
    %These commands keep the window from automatically rescaling in funny ways.
    axis(Limits);
    axis('equal');
    axis manual;
    axis off;
    
    
else    %THEN RUNNING STOP ACTION
    
    hold on;
    
    %Plot Cart
    x = Cart(1) - 0.5*Cart_Width;
    y = -0.5*Cart_Height;
    w = Cart_Width;
    h = Cart_Height;
    rectangle('Position',[x,y,w,h],'LineWidth',2);
    
    %Plot Pendulum
    Rod_X = [Cart(1), Bob(1)];
    Rod_Y = [Cart(2), Bob(2)];
    plot(Rod_X,Rod_Y,'k-','LineWidth',2)
    
    %Plot Bob and hinge
    plot(Bob(1),Bob(2),'k.','MarkerSize',30)
    plot(Cart(1),Cart(2),'k.','MarkerSize',30)
    
    %Plot Rails
    plot([Limits(1) Limits(2)],-0.5*Cart_Height*[1,1],'k-','LineWidth',2)
    
    %Plot trace of the Bob's path:
    plot(param.Bob(1,:),param.Bob(2,:),'k:','LineWidth',1)
    
    
    %These commands keep the window from automatically rescaling in funny ways.
    axis(Limits);
    axis('equal');
    axis manual;
    axis off;
    
end
end