function plotTraj(traj,config)
% This function plots a trajectory

markerSize = 25;
lineWidth = 2;

t = traj.time;
tt = linspace(t(1),t(end),150);

x = traj.state;
xx = traj.interp.state(tt);

u = traj.control;
uu = traj.interp.control(tt);

subplot(2,3,1); hold on;
plot(t,x(1,:),'k.','MarkerSize',markerSize);
plot(tt,xx(1,:),'b-','LineWidth',lineWidth);
xlabel('t')
ylabel('x')

subplot(2,3,2); hold on;
plot(t,x(2,:),'k.','MarkerSize',markerSize);
plot(tt,xx(2,:),'b-','LineWidth',lineWidth);
xlabel('t')
ylabel('q')

subplot(2,3,3); hold on;
plot(t,u(1,:),'k.','MarkerSize',markerSize);
plot(tt,uu(1,:),'b-','LineWidth',lineWidth);
xlabel('t')
ylabel('F')

subplot(2,3,4); hold on;
plot(t,x(3,:),'k.','MarkerSize',markerSize);
plot(tt,xx(3,:),'b-','LineWidth',lineWidth);
xlabel('t')
ylabel('dx')

subplot(2,3,5); hold on;
plot(t,x(4,:),'k.','MarkerSize',markerSize);
plot(tt,xx(4,:),'b-','LineWidth',lineWidth);
xlabel('t')
ylabel('dq')

end