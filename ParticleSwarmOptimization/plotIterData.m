function plotIterData(soln,i)
%
% This function plots all of the points for a given iteration, assuming
% that the objective function is 2 dimensional with the origin at zero.
%

g = soln.log.Gx(:,1,i);  %global best
f = soln.log.Gf(1,1,i);  %best objective function value
x = soln.log.X(:,:,i);  %local search point
p = soln.log.Lx(:,:,i);  %local best point

clf; hold on;
plot(0,0,'ko','MarkerSize',20,'LineWidth',3);
plot(g(1),g(2),'g.','MarkerSize',30);
plot(x(1,:),x(2,:),'k.','MarkerSize',15);
plot(p(1,:),p(2,:),'b.','MarkerSize',8);
title(sprintf('Objective function: %4.4f',f));

axis([soln.problem.xLow(1),soln.problem.xUpp(1),soln.problem.xLow(2),soln.problem.xUpp(2)]);

end