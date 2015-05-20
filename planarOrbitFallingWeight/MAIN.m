% MAIN
%
% Simulates:
%
% Problem Description:
%   Suppose that you have two point-masses that are connected by a
%   mass-less inextensible string. One mass is able to travel around
%   horizontal planar surface. The second mass is hanging from the point
%   where the string passes through a small hole in the surface.
%
% Intesting things:
%   
% --> If you restict the motion to be along a single axis...
%       1) Why is the velocity a triangle wave?
%       2) Why does the system seem to loose energy in this special case?
%
% --> Can you get circular motion?
%

% Parameters
p.m1 = 1.0;
p.m2 = 1.0;
p.g = 9;

% Initial condition
x0 = 1.0;
y0 = 0;
dx0 = 0;
dy0 = 3;
s0 = [x0;y0;dx0;dy0];

% Simulate:
tSpan = [0,30];
options = odeset(...
    'AbsTol',1e-8,...
    'RelTol',1e-8);
userFun = @(t,s)dynamics(s,p);
[t,s] = ode45(userFun,tSpan,s0);
s = s';

% Extract states:
x = s(1,:);
y = s(2,:);
dx = s(3,:);
dy = s(4,:);

% Plot:
figure(1); clf; hold on;
subplot(2,2,1);
plot(t,x);
xlabel('t');
ylabel('x');
subplot(2,2,2);
plot(t,y);
xlabel('t');
ylabel('y');
subplot(2,2,3);
plot(t,dx);
xlabel('t');
ylabel('dx');
subplot(2,2,4);
plot(t,dy);
xlabel('t');
ylabel('dy');

figure(2); clf; hold on;
plot(x,y);
plot(0,0,'k.','MarkerSize',10);
xlabel('x')
ylabel('y')
axis equal;
