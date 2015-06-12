% Derive_doublePendulum.m
%
% This script uses Matlab symbolic toolbox to derive the equations of
% motion for a double pendulum with point-masses at the end of each link
%

syms q1 q2 dq1 dq2 ddq1 ddq2 'real' % states
syms u 'real'  % torque between the links
syms m1 m2 g l1 l2 'real'   % physical parameters

%%% Unit vectors:
i = sym([1;0]);
j = sym([0;1]);

e1 = cos(q1)*(-j) + sin(q1)*(i);   %shoulder joint -> elbow joint
e2 = cos(q2)*(-j) + sin(q2)*(i);   %elbow joint -> tip of second link

%%%% State vectors:
z = [q1; q2; dq1; dq2];
dz = [dq1; dq2; ddq1; ddq2];

%%%% Kinematics:
p1 = l1*e1;
p2 = p1 + l2*e2;

dp1 = jacobian(p1,z)*dz;
dp2 = jacobian(p2,z)*dz;

ddp1 = jacobian(dp1,z)*dz;
ddp2 = jacobian(dp2,z)*dz;

%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%%%% Angular momentum balance of system about shoulder joint:
sumTorque1 = cross2d(p1,-m1*g*j) + cross2d(p2, -m2*g*j);
sumInertial1 = cross2d(p1,m1*ddp1) + cross2d(p2, m2*ddp2);
eqn1 = sumTorque1-sumInertial1;

%%%% Angular momentum balance of second link about elbow:
sumTorque2 = cross2d(p2-p1, -m2*g*j) + u;
sumInertial2 = cross2d(p2-p1, m2*ddp2);
eqn2 = sumTorque2-sumInertial2;

%%%% Extract dynamics matricies:
eqns = [eqn1;eqn2];

% D*[ddq1;ddq2] = tmp == B*u - G
[D,tmp] = equationsToMatrix(eqns,[ddq1;ddq2]);
[B,G] = equationsToMatrix(tmp,u);

error('SOMETHING DIDNT WORK HERE')

%%%% Check the result:
check = simplify(D*[ddq1;ddq2] + G - B*u);  % check should be zeros

