% MAIN.m  --  Three Mass Periodic Orbits
%
% This script uses bvp4c to find periodic trajectories for three
% point-masses in 3D space with newtonian gravity.
%
% The solution to this problem was discovered by Montgomery in
%
%
%
% BVP4C does not seem to work well for this problem since you cannot apply
% constraints to it. The basic problem is that there are two strong local
% minimums in the problem that are undesirable. The first is that all
% masses are right on top of each other (boring). The second occurs when
% the  masses are relatively far away, which causes the forces acting on
% them to be small... enough so that the they are within the tolerances
% allowed by bvp4c;
%


% Physical Parameters
P.G = 1.0;   %Gravity Constant
P.m1 = 1.0;  %mass one
P.m2 = 1.0;  %mass two
P.m3 = 1.0;  %mass three

% Create function handles:
odeFun = @(t,x)( dynamics(x,P) );
bcFun = @(x0,xF)( boundaryConditions(x0,xF) );

% Create an initial guess by simulation
tSpan = [0, 1];
x1 = [1; 0; -1];
x2 = [0; 1; 0];
v1 = [0; 0; 1];
v2 = [-1; 0; 0];
X0 = [x1; x2; v1; v2];
sol = ode45(odeFun,tSpan,X0);

% Build the special initialization struct:
solnInit.solver = 'bvpinit';
solnInit.x = linspace(tSpan(1), tSpan(2), 250);
solnInit.y = deval(sol, solnInit.x);
solnInit.yinit = X0;

% Options:
options = bvpset(...
    'RelTol',1e-3,...
    'AbsTol',1e-3,...
    'Stats','on',...
    'Vectorized', 'on');

% Solve with 4th-order collocation:
soln = bvp4c(odeFun,bcFun,solnInit,options);

% High-order interpolation of the solution:
t = linspace(soln.x(1), soln.x(end), 100);
X = deval(soln,t);

% Figure out the position of the third particle by momentum conservation:
X1 = X(1:3,:);
X2 = X(4:6,:);
X3 = -(X1*P.m1 + X2*P.m2)/P.m3;
V1 = X(7:9,:);
V2 = X(10:12,:);
V3 = -(V1*P.m1 + V2*P.m2)/P.m3;

%Unpack the state for plotting:
x1 = X1(1,:);
y1 = X1(2,:);
z1 = X1(3,:);
x2 = X2(1,:);
y2 = X2(2,:);
z2 = X2(3,:);
x3 = X3(1,:);
y3 = X3(2,:);
z3 = X3(3,:);
dx1 = V1(1,:);
dy1 = V1(2,:);
dz1 = V1(3,:);
dx2 = V2(1,:);
dy2 = V2(2,:);
dz2 = V2(3,:);
dx3 = V3(1,:);
dy3 = V3(2,:);
dz3 = V3(3,:);

% Plot the solution:

figure(1); clf; hold on;
plot3(x1,y1,z1,'r')
plot3(x2,y2,z2,'g')
plot3(x3,y3,z3,'b')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal;
legend('1','2','3');

figure(2); clf;
subplot(3,2,1); hold on;
plot(t,x1,'r');
plot(t,x2,'g');
plot(t,x3,'b');
xlabel('t');
ylabel('x');
subplot(3,2,3); hold on;
plot(t,y1,'r');
plot(t,y2,'g');
plot(t,y3,'b');
xlabel('t');
ylabel('y');
subplot(3,2,5); hold on;
plot(t,z1,'r');
plot(t,z2,'g');
plot(t,z3,'b');
xlabel('t');
ylabel('z');
subplot(3,2,2); hold on;
plot(t,dx1,'r');
plot(t,dx2,'g');
plot(t,dx3,'b');
xlabel('t');
ylabel('dx');
subplot(3,2,4); hold on;
plot(t,dy1,'r');
plot(t,dy2,'g');
plot(t,dy3,'b');
xlabel('t');
ylabel('dy');
subplot(3,2,6); hold on;
plot(t,dz1,'r');
plot(t,dz2,'g');
plot(t,dz3,'b');
xlabel('t');
ylabel('dz');



