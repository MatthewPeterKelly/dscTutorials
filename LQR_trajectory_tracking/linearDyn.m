function [A, B] = linearDyn(t,z,u,P)

%Pendulum with a control input orthogonal to gravity

m = P.m;
c = P.c;
k = P.k;

x = z(1,:);
v = z(2,:);
n = length(x);

A = zeros(2,2,n);
B = zeros(2,1,n);

A(1,1,:) = -(u.*sin(x)-k*cos(x))/m;
A(1,2,:) = -c/m;
A(2,1,:) = 0;
A(2,2,:) = 1;

B(1,1,:) = cos(x);
B(2,1,:) = 0;

end