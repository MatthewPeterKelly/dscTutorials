% Derive Equations --  Dynamics Function Gradients
%
% This script derives the the gradients of the dynamics of a double
% pendulum, using two methods to check math.
%
%

clc; clear;

syms q1 dq1 ddq1 'real' % States
syms q2 dq2 ddq2 'real' % States
syms u0 u1 u2 'real'  %Inputs  (gravity, shoulder, elbow)

nx = 2;  %Dimension of the configuration space
nu = 3;
nz = nx+nx+nu;

q = [q1;q2];
dq = [dq1; dq2];
ddq = [ddq1; ddq2];
u = [u0;u1;u2];

% Unit vectors
i = [1;0];
j = [0;1];

% Position of point mass (assume unit length links)
p1 = cos(q1)*i + sin(q1)*j;
p2 = p1 + cos(q2)*i + sin(q2)*j;

% Neat trick to compute derivatives using the chain rule
derivative = @(in)( jacobian(in,[q;dq])*[dq;ddq] );

% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

% Kinematics:
dp1 = derivative(p1); ddp1 = derivative(dp1);
dp2 = derivative(p2); ddp2 = derivative(dp2);

% Angular momentum balance of both links about base joint
eqnTorques1 = cross2d(p1,-u0*j) + cross2d(p2,-u0*j) + u1;  %unit mass and length
eqnInertia1 = cross2d(p1,ddp1) + cross2d(p2,ddp2);  

% Angular momentum balance of outer link about middle joint 
eqnTorques2 = cross2d(p2-p1,-u0*j) + u2;  %unit mass and length
eqnInertia2 = cross2d(p2-p1,ddp2);  

% Collect equations:
eqns = [eqnTorques1-eqnInertia1; eqnTorques2-eqnInertia2];
[A, b] = equationsToMatrix(eqns,ddq);
A = simplify(A); b = simplify(b);

% Compute the jacobian of the mass matrix and generalized force
[Az, bz] = symbolicGradients(A,b,q,dq,u);

% Compute the jacobian of the analytic inverse:
F = simplify(A\b);
z = [q;dq;u];
Fz = reshape(jacobian(F,z),2*nz,1);

% This is a stupid hack to force the symbolic toolbox to vectorize the
% output of its function. Otherwise, matlab tries to put scalar "1" and "0"
% enteries into an entire row of the matrix.
syms empty 'real';  %Dummy placeholder to force vectorization
A = A + empty;
b = b + empty;
Az = Az + empty;
bz = bz + empty;
F = F + empty;
Fz = Fz + empty;

% Write function files:
matlabFunction(reshape(A,nx*nx,1), b, Az, bz,...
    'file','autoGen_dynamicsNumeric.m',...
    'vars',{q1, q2, dq1, dq2, u0, u1, u2, empty},...
    'outputs',{'A', 'b', 'Az', 'bz'});

matlabFunction(F, Fz,...
    'file','autoGen_dynamicsAnalytic.m',...
    'vars',{q1, q2, dq1, dq2, u0, u1, u2, empty},...
    'outputs',{'F', 'Fz'});


    