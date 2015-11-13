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

%Don't clear the figure each time
param.clearFigure = false;

%Pass the trace of the Bob's path to the plotting function.
nPts = 500;
traceTime = linspace(t(1),t(end),nPts);
traceState = interp1(t',Bob',traceTime','pchip','extrap')';
param.Bob = traceState;


%Plotting:
clf; 
nFrames = 20;
frameTime = linspace(t(1),t(end),nFrames);
frameState = interp1(t',x',frameTime','pchip','extrap')';
for i=1:nFrames
   plotPendulumCart(frameTime(i),frameState(:,i),param); 
end


