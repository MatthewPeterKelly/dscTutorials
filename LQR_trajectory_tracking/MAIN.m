%MAIN  --  LQR trajectory tracking

%This script generates a trajectory, and then solves a few lqr controllers
%along the trajectory. It then runs a simulation to see how well the
%controllers can track the reference.

clc; clear;

P.m = 1;
P.c = 0.2;
P.k = 1;
P.w = 1;

P.IC = [0;0];
P.N = 10;   %Number of samples for lqr linearization
P.NN = 250;  %Number of samples in exact trajectory
P.Tspan = [0,10];

P.xMax = 1;
P.vMax = 10;
P.uMax = 1;

%Create a trajectory to track
[T,Nom] = getTrajectory(P);

%Solve the time-varying LQR problem:
T = solveLQR(T);
P.T = T;

%Simulate the close-loop system:
IC = P.IC + [0.02;0.04];
Sim = runSimulation(T,IC,P);

%Make a nice plot of the trajectory:
figure(1); clf;

subplot(3,2,1); hold on;
plot(Nom.t,Nom.Z(1,:),'k-')
plot(Sim.t,Sim.Z(1,:),'r-')
title('Position')
subplot(3,2,3); hold on;
plot(Nom.t,Nom.Z(2,:),'k-')
plot(Sim.t,Sim.Z(2,:),'r-')
title('Velocity')
subplot(3,2,5); hold on;
plot(Nom.t,Nom.u,'k-')
plot(Sim.t,Sim.u,'r-')
title('Input')

subplot(3,2,[2,4,6]); hold on;
plot(Nom.Z(1,:),Nom.Z(2,:),'k-')
plot(Sim.Z(1,:),Sim.Z(2,:),'r-')
title('State Space')