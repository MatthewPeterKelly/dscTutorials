function plotSoln(traj, target, param)
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

plotBackground(target,param);  hold on;

% Plot the trajectory
x = traj.x;         y = traj.y;
dx = traj.dx;       dy = traj.dy;
plot(x,y,'k-','LineWidth',3); 

%%% Add labels and title
objVal = objective(dx(1),dy(1));
title(sprintf('Trajectory via %s   (cost = %6.3f)',traj.method ,objVal));
xlabel('Y  Position')
ylabel('X  Position')
hold off;

end