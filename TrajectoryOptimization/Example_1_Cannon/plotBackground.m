function plotAxis = plotBackground(target,param)

hold on;

%%% Figure out the axis for the figure:
pixelDim = param.diagnostics.gifPixelDim;
axisRatio = pixelDim(2)/pixelDim(1);
xBnd = [-0.2*target.x, 1.2*target.x];
yBnd = xBnd*axisRatio;
plotAxis = [xBnd,yBnd];

%%% Set the color for the background and size of figure
set(gca,'Color',0.7*[1 1 1]);  %Grey
position = get(gcf,'Position'); 
position(3:4) = pixelDim;
set(gcf,'Position',position);

%%% Plot the ground
xGround = [-0.2*target.x, 1.2*target.x];
yGround = [0, 0];
brown = [0.5, 0.2, 0.1];  %Color for ground
plot(xGround, yGround,'color',brown,'LineWidth',6);

%%% Plot the tree, start position, and target position
drawTree(0.6*target.x, 0, 1);  %Plot a tree for scale
plot(0,0,'k.','MarkerSize',35);   %Start
plot(target.x,target.y,'ko','LineWidth',4,'MarkerSize',14)   %Finish

%%% Axis Stuff
xlabel('Horizontal Position')
ylabel('Vertical Position')
axis equal; axis(plotAxis); drawnow; hold off;

end