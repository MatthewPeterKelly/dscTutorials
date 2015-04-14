function dx = pendulumSystem(x,A,P,grid,dyn)
% dx = pendulumSystem(x,A,P,pendulum)
%
% This function computes the closed loop dynamics of the pendulum, using
% the optimal controller from the MDP solve
%

th = x(1,:);
w = x(2,:);

u = pendulumController(x,A,P,grid);
dw = pendulumDynamics(th,w,u,dyn);

dx = [w;dw];

end