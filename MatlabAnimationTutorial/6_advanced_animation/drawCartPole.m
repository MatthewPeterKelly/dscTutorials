function drawCartPole(time,pos,extents,p)
% drawCartPole(time,pos,extents,p)
%
% INPUTS:
%   time = [scalar] = current time in the simulation
%   pos = [4, 1] = [x1;y1;x2;y2]; = position of [cart; pole]
%   extents = [xLow, xUpp, yLow, yUpp] = boundary of the draw window
%
% OUTPUTS:
%   --> A pretty drawing of a cart-pole
%

% Make plot objects global so that they don't get erased
global railHandle cartHandle poleHandle bobHandle starHandle;

hold on;    %  <-- This is important!

cartColor = [0.2, 0.7, 0.2];   % [R, G, B]
poleColor = [0.3, 0.2, 0.7];

%%%% Unpack the positions:
x1 = pos(1);
y1 = pos(2);
x2 = pos(3);
y2 = pos(4);

% Title and simulation time:
title(sprintf('t = %2.2f%',time));


% Draw the rail that the cart-pole travels on
if isempty(railHandle)
    railHandle = plot(extents(1:2),[0,0],'k-','LineWidth',2);
end


% Draw the cart:
if isempty(cartHandle)
    cartHandle =  rectangle(...
        'Position',[x1-0.5*p.w, y1-0.5*p.h, p.w, p.h],...
        'Curvature',0.5*[p.h,p.w]/p.l,...
        'LineWidth',2,...
        'FaceColor',cartColor,...
        'EdgeColor',0.5*cartColor);  %Darker version of color
else
    set(cartHandle,...
        'Position',[x1-0.5*p.w, y1-0.5*p.h, p.w, p.h]);
end


% Draw the pole:
if isempty(poleHandle)
    poleHandle = plot([x1,x2], [y1, y2],...
        'LineWidth',4,...
        'Color',0.8*poleColor);
else
    set(poleHandle,...
        'xData',[x1,x2],...
        'yData',[y1,y2]);
end


% Draw the bob of the pendulum:
if isempty(bobHandle)
    bobHandle = rectangle(...
        'Position',[x2-p.r, y2-p.r, 2*p.r, 2*p.r],...
        'Curvature',[1,1],...   % <-- Draws a circle...
        'LineWidth',2,...
        'FaceColor',poleColor,...
        'EdgeColor',0.5*poleColor);  %Darker version of color
else
    set(bobHandle,...
        'Position',[x2-p.r, y2-p.r, 2*p.r, 2*p.r]);
end


% Draw a star for fun:
if isempty(starHandle)
    starHandle = patch(...
        x1 + p.star(1,:),...
        y1 + p.star(2,:),...
        'r');
else
    set(starHandle,...
        'XData',x1 + p.star(1,:),...
        'YData',y1 + p.star(2,:));
end


% Format the axis so things look right:
axis equal; axis(extents); axis off;      %  <-- Order is important here

% Push the draw commands through the plot buffer
drawnow;

end