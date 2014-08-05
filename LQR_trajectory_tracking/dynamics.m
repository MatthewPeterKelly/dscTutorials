function dZ = dynamics(t,z,u,P)

%Pendulum with a control input orthogonal to gravity

m = P.m;
c = P.c;
k = P.k;

x = z(1,:);
v = z(2,:);

dx = v;
dv =(u.*cos(x) - k*sin(x) - c*v)/m;

dZ = [dx;dv];

end