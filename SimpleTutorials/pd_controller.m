% pd_controller.m
%
% This function simulates a pd controller to move around a point mass


m = 1.0;   %Mass

wn = 2.5;   %Natural frequency
xi = 0.8;   %Damping ratio

kp = m*wn*wn;      % proportional gain
kd = 2*m*wn*xi;    % derivative gain

xRef = 0.0;    % position target
vRef = 0.0;    % velocity target

xInit = 1.0;  % initial position
vInit = 0.0;  % initial velocity

% Control:
%       f = kp*(xRef - x) + kd*(vRef-v);

f = @(x,v)( kp*(xRef - x) + kd*(vRef-v) );


% Dynamics: 
%       f = ma
%
%       dx = v;
%       dv = f/m;
%
%       z = [x;v];
%       dz = [v;f/m];

dz = @(t,z)( [z(2); f(z(1),z(2))/m] );


% Simulation:
tSpan = [0, 9/(xi*wn)];
z0 = [xInit; vInit];
sol = ode45(dz,tSpan,z0);

% Plotting:
t = linspace(tSpan(1), tSpan(2), 150);
z = deval(sol,t);
x = z(1,:);
v = z(2,:);
u = f(x,v);

figure(1); clf;

subplot(3,1,1);
plot(t,x)
xlabel('time (s)');
ylabel('position (m)');
title('Controller Response')

subplot(3,1,2);
plot(t,v)
xlabel('time (s)')
ylabel('velocity (m)');

subplot(3,1,3);
plot(t,u)
xlabel('time (s)')
ylabel('force (N)')





