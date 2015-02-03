function plotTraj(traj, target)
% Plots a cannon ball trajectory (x,y), with speed trajectory (dx,dy)
%
% traj.x = [1 x n] horizontal position vector
% traj.y = [1 x n] vertical position vector
% traj.dx = [1 x n] horizontal speed vector
% traj.dy = [1 x n] vertical speed vector
%
% target.x = horizontal target
% target.y = vertical target
%

%%% Unpack the trajectory
x = traj.x;         y = traj.y;
dx = traj.dx;       dy = traj.dy;

%%% Define the ground
xGround = [-0.1*target.x, 1.1*target.x];
yGround = [0, 0];

%%% Figure out the extents of the plot window
yMin = min(y); yMax = max(y); yDif = yMax-yMin;
yAxis = [yMin - 0.1*yDif, yMax+0.1*yDif];
clf; hold on; axis equal; axis([xGround,yAxis]); 

%%% Plot the background
plot(xGround, yGround,'color',[0.5, 0.2, 0.1],'LineWidth',4);
drawTree(0.6*target.x, 0, 1);  %Plot a tree for scale
plot(0,0,'b.','MarkerSize',35);   %Start
plot(target.x,target.y,'rx','LineWidth',4,'MarkerSize',14)   %Finish

% Plot the trajectory
plot(x,y,'k-','LineWidth',3); 

%%% Add labels and title
title(sprintf('Cannon Ball Trajectory:    (cost = %6.3f)',dx(end)^2 + dy(end)^2));
xlabel('Y  Position')
ylabel('X  Position')
hold off;

end