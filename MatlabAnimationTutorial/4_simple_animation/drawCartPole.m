function drawCartPole(time,pos,extents)
% drawCartPole(time,pos,extents)
%
% INPUTS:
%   time = [scalar] = current time in the simulation
%   pos = [4, 1] = [x1;y1;x2;y2]; = position of [cart; pole]
%   extents = [xLow, xUpp, yLow, yUpp] = boundary of the draw window
%
% OUTPUTS:
%   --> A pretty drawing of a cart-pole
%

%%%% Unpack the positions:
x1 = pos(1);
y1 = pos(2);
x2 = pos(3);
y2 = pos(4);

% Title and simulation time:
title(sprintf('t = %2.2f%',time));

% Draw the rail that the cart-pole travels on
plot(extents(1:2),[0,0],'k-','LineWidth',2);

% Draw the cart:
plot(x1, y1, 'bs','MarkerSize',30,'LineWidth',5);

% Draw the pole:
plot([x1,x2], [y1, y2], 'r-','LineWidth',2);

% Draw the bob of the pendulum:
plot(x2, y2, 'ro','MarkerSize',22,'LineWidth',4);

% Format the axis so things look right:
axis equal; axis(extents); axis off;      %  <-- Order is important here

end