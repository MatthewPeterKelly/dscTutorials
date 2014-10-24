%MAIN - run a simulation of the rocket
clear; clc;

%As of now, it does nearly an orbit before crashing into the surface of the
%planet... 

P.m = 2;   %(kg) mass of rocket
P.l = 1;    %(m) length of rocket
P.d = 0.1*P.l;   %(m) eccentricity of engines
P.rPlanet = 100;   %(m) radius of the planet
P.G = 1;  %(Nm^2/kg^2) gravity constant
P.mPlanet = 1000;  %(kg) mass of planet

tSpan = [0,500];  %Simulation duration

q0 = [1.5*P.rPlanet; 0;pi/2];
dq0 = [0;0.015;0];
z0 = [q0;dq0];

options = odeset('MaxStep',diff(tSpan)/100);

sol = ode45(@(t,z)rhs(t,z,P),tSpan,z0,options);

t = sol.x;
r = sol.y(1,:);
th1 = sol.y(2,:);
th2 = sol.y(3,:);
dr = sol.y(4,:);
dth1 = sol.y(5,:);

e1 = [-sin(th1);cos(th1)];
n1 = [-cos(th1); -sin(th1)];

p = [r.*e1(1,:); r.*e1(2,:)];
dp = [dr.*e1(1,:) + r.*dth1.*n1(1,:); dr.*e1(2,:) + r.*dth1.*n1(2,:)];
speed = sqrt(sum(dp.^2,1));

q = sol.y(1:3,:);
dq = sol.y(4:6,:);
u = controller(q,dq,P);
u1 = u(1,:);
u2 = u(2,:);


figure(303); clf;
rectangle('Position',P.rPlanet*[-1,-1,2,2],'Curvature',[1,1],'LineWidth',2);
hold on;
plot(p(1,:),p(2,:),'r-','LineWidth',2);
plot(0,0,'k.','MarkerSize',10)
axis equal; axis off;
title('Trajectory')

figure(304); clf;
subplot(3,1,1); 
plot(t,u1+u2,'r-','LineWidth',2);
ylabel('Thrust')
subplot(3,1,2); 
plot(t,u1-u2,'b-','LineWidth',2);
ylabel('Torque')
subplot(3,1,3)
plot(t,speed,'k-','LineWidth',2);
title('Speed');

figure(305); clf;
subplot(3,1,1);
plot(t,r,'LineWidth',2);
ylabel('Radius')
subplot(3,1,2);
plot(t,th1,'LineWidth',2);
ylabel('Angle Coordinate')
subplot(3,1,3);
plot(t,th2-th1,'LineWidth',2);
ylabel('Attitude')

figure(303);
