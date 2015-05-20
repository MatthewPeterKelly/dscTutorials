function ds = dynamics(s,p)
% ds = dynamics(s,p)
%
% This function computes the dynamics for a point mass that is travelling
% on a frictionless plane, acted on by another point mass that is hanging
% below the origin. They are connected by an inextensible massless string
%

x = s(1,:);
y = s(2,:);
dx = s(3,:);
dy = s(4,:);

[ddx,ddy] = autoGen_dynamics(x,y,dx,dy,p.m1,p.m2,p.g);

ds = [dx;dy;ddx;ddy];

end