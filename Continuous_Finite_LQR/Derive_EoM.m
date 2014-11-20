%Drive Equations of Motion

%Non-linear 2-dimensional system, with one input

syms x y u 'real'

% % dx = y*sin(x) + y*y;
% % dy = x*y + x*cos(x)+ u*cos(x);

dx = sin(y)+1-cos(u);
dy = sin(x)+sin(u);

z = [x;y];
f = [dx;dy];

A = jacobian(f,z);
B = jacobian(f,u);

%Write the dynamics to a file
matlabFunction(f(1),f(2),'file','dynamics.m','vars',{'x','y','u'},'outputs',{'dx','dy'});

%Write the linearized dynamics to a file
matlabFunction(A,B,'file','getLinSys.m','vars',{'x','y','u'},'outputs',{'A','B'});


