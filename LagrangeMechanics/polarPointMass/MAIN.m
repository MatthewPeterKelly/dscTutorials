% MAIN  --  Polar Point Mass
%
% This script runs a simulation of the polar point mass, with a controller
% that is working to hold the mass at a fixed distance from the origin
%

P.m = 1;
P.g = 9.81;
P.l = 1;
P.freq = 3*sqrt(P.g/P.l);   %Response frequence in controller
P.damp = 1.0;   %Damping ratio in controller
P.eNom = 2.5*P.m*P.g*P.l;

% z = [r; th; dr; dth];

z0min = [0.5*P.l; -pi; -0.2; -0.2];
z0max = [1.5*P.l; pi;  0.2;  0.2];


z0 = z0min + (z0max-z0min).*rand(4,1);

userFunc = @(t,z)dynamics(t,z,controller(z,P),P);

tSpan = [0;3];

sol = ode45(userFunc,tSpan,z0);

t = linspace(tSpan(1),tSpan(2),500);
z = deval(sol,t);
u = controller(z,P);

figure(234); clf;

subplot(3,2,1);
plot(t,z(1,:))
ylabel('r')

subplot(3,2,3);
plot(t,z(3,:))
ylabel('dr')

subplot(3,2,5);
plot(t,u)
ylabel('u')

subplot(3,2,2);
plot(t,z(2,:))
ylabel('th')

subplot(3,2,4);
plot(t,z(4,:))
ylabel('dth')

