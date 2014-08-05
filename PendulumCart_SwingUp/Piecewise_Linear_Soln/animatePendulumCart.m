%Unpack the data
pos = State(1,:);
angle = State(3,:);

%Position of the cart over time:
Cart = [pos;zeros(size(pos))];

%Position of the pendulum bob over time:  (Assume that length==1)
Bob = Cart + [-sin(angle);cos(angle)];

%Pack up for plotting:
x = [Cart;Bob];
t = Time;

%Figure out the extents of the axis
horizontalAll = [Cart(1,:), Bob(1,:)];
verticalAll = [Cart(2,:), Bob(2,:)];
param.axis = [min(horizontalAll),max(horizontalAll),...
  min(verticalAll),max(verticalAll)];
param.clearFigure = true;

%Pass the trace of the Bob's path to the plotting function.
param.Bob = Bob;

%Set up parameters for animation:
P.plotFunc = @(t,x)plotPendulumCart(t,x,param);
P.speed = 0.25;
P.figNum = gcf;

%Call the animation function
Animate(t,x,P)