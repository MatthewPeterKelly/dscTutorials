% Derive_acrobot.m
%
% This script derives the equations of motion for the simple acrobot robot:
% a double pendulum with a motor at the elbow joint. 
%

syms q1 q2 dq1 dq2 ddq1 ddq2 'real'   % states
syms u 'real' % actuation
syms m1 m2 g l1 l2 'real' % physical parameters

%%%% Unit vectors:
i = sym([1;0]);
j = sym([0;1]);

e1 = cos(q1)*(-j) + sin(q1)*(i);    % shoulder -> elbow
e2 = cos(q2)*(-j) + sin(q2)*(i);    % elbow -> wrist

%%%% State vectors:
z = [q1;q2;dq1;dq2];
dz = [dq1;dq2;ddq1;ddq2];

%%%% Kinematics:
p1 = l1*e1;
p2 = p1 + l2*e2;

dp1 = jacobian(p1,z)*dz;  %Chain rule to get velocity of elbow joint
dp2 = jacobian(p2,z)*dz; 

ddp1 = jacobian(dp1,z)*dz;  
ddp2 = jacobian(dp2,z)*dz; 

%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%%%% Angular momentum balance of system about shoulder joint:
sumTorques1 = cross2d(p1,-m1*g*j) + cross2d(p2,-m2*g*j);
sumInertial1 = cross2d(p1,m1*ddp1) + cross2d(p2,m2*ddp2);
eqn1 = sumTorques1-sumInertial1;

%%%% Angular momentum balance of outer link about elbow joint:
sumTorques2 = cross2d(p2-p1,-m2*g*j) + u;
sumInertial2 = cross2d(p2-p1,m2*ddp2);
eqn2 = sumTorques2-sumInertial2;

%%%% Write out dynamics in matrix form:   D*ddq + G = B*u
ddq = [ddq1;ddq2];
eqns = [eqn1;eqn2];

% D*ddq = tmp = B*u - G
[D,tmp] = equationsToMatrix(eqns,ddq); %check: simplify(eqns - (D*ddq-tmp))
[B,G] = equationsToMatrix(tmp,u); %check: simplify(eqns - (D*ddq-(B*u-G)))

%%%% Generate an optimized matlab function for dynamics:
matlabFunction(D,G,B,...
    'file','autoGen_acrobotDynamics.m',...
    'vars',{q1,q2,dq1,dq2,m1,m2,g,l1,l2},...
    'outputs',{'D','G','B'});

%%%% Compute the energy of the system:
U = m1*g*dot(p1,j) + m2*g*dot(p2,j);   %Potential Energy
T = 0.5*m1*dot(dp1,dp1) + 0.5*m2*dot(dp2,dp2);   %Kinetic Energy

%%%% Generate an optimized matlab function for energy:
matlabFunction(U,T,...
    'file','autoGen_acrobotEnergy.m',...
    'vars',{q1,q2,dq1,dq2,m1,m2,g,l1,l2},...
    'outputs',{'U','T'});

%%%% Generate a function for computing the kinematics:
matlabFunction(p1,p2,dp1,dp2,...
    'file','autoGen_acrobotKinematics.m',...
    'vars',{q1,q2,dq1,dq2,l1,l2},...
    'outputs',{'p1','p2','dp1','dp2'});


