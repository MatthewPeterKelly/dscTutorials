function ddth = pendulumDynamics(th,dth,u,p)
% ddth = pendulumDynamics(th,dth,u,p) 
%
% This function computes the second-order dynamics of a driven damped
% pendulum.
%

ddth = (p.g/p.l)*sin(th) + (-p.c*dth + u)/(p.m*p.l*p.l);

end