%MAIN
%
% This script is used for testing the continuous, finite horizon, LQR
% solver that I'm working on.
%
%
clear; clc;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Draw the dynamical system                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

n = 13;
x = linspace(-1,1,n);
y = linspace(-1,1,n);

[xx,yy] = meshgrid(x,y);

[dxx, dyy] = dynamics(xx,yy,zeros(n,n));

figure(1); clf; hold on;
title('Dynamical System');
quiver(xx,yy,dxx,dyy);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Compute a reference trajectory                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

options = odeset();
options.Events = @eventFunc;
tSpan = [0,-10];

control = @(t) -0.25*sin(t);

userFunc = @(t,z)rhs(t,z,control(t));
z0 = [0;0];

sol = ode45(userFunc,tSpan,z0,options);

nTime = 70;
tSol = linspace(sol.x(end),sol.x(1),nTime);
zSol = deval(sol,tSol);
xSol = zSol(1,:);
ySol = zSol(2,:);
uSol = control(tSol);
tSpan = [tSol(1),tSol(end)];

plot(xSol,ySol,'r-','LineWidth',3);
plot(0,0,'ro','MarkerSize',10,'LineWidth',2)
axis(1.2*[-1,1,-1,1]); axis equal; axis off

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Store the reference trajectory                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%We will store each trajectory as a polynomial. For low-order fits, say
%less than 10th order, Matlab's polyval and polyfit are good. For
%higher-order fit's, it is best to use different method, based on
%barycentric interpolation. Google chebyfun.

nFit = 5;  %Order of polynomial fitting
xFit = polyfit(tSol,xSol,nFit);
yFit = polyfit(tSol,ySol,nFit);
uFit = polyfit(tSol,uSol,nFit);

figure(2); clf;
subplot(3,1,1); hold on;
plot(tSol,xSol,'k.','MarkerSize',15)
plot(tSol,polyval(xFit,tSol),'r-')
xlim(tSpan);
ylabel('x')
title('Polynomial approximation of trajectory')
subplot(3,1,2); hold on;
plot(tSol,ySol,'k.','MarkerSize',15)
plot(tSol,polyval(yFit,tSol),'r-')
xlim(tSpan);
ylabel('y')
subplot(3,1,3); hold on;
plot(tSol,uSol,'k.','MarkerSize',15)
plot(tSol,polyval(uFit,tSol),'r-')
xlim(tSpan);
ylabel('u')
xlabel('t')


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Compute LQR Gains along trajectory                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Function handle for getting linearized system dynamics
linSys = @(t)getLinTraj(t,xFit,yFit,uFit);

Q = eye(2);  % Running cost on state
R = 1;       % Running cost on input
F = eye(2);  % Terminal cost on state
tol = 1e-6;  % Accuracy of ricatti propagation

Soln = trajectoryLqr(tSol,linSys,Q,R,F,tol);

K = reshape([Soln.K],2,nTime);
kxFit = polyfit(tSol,K(1,:),nFit);
kyFit = polyfit(tSol,K(2,:),nFit);

figure(3); clf;
subplot(2,1,1); hold on;
plot(tSol,K(1,:),'k.','MarkerSize',15)
plot(tSol,polyval(kxFit,tSol),'r-')
xlim(tSpan);
ylabel('Kx')
title('Polynomial approximation of lqr gains')
subplot(2,1,2); hold on;
plot(tSol,K(2,:),'k.','MarkerSize',15)
plot(tSol,polyval(kyFit,tSol),'r-')
xlim(tSpan);
ylabel('Ky')
xlabel('t')

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Compare to infinite-horizon LQR gains                %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

ssSoln(nTime).t = 0;
ssSoln(nTime).K = zeros(1,2);
ssSoln(nTime).S = zeros(2);
ssSoln(nTime).E = zeros(2,1);
for i=1:nTime
    tNow = tSol(i);
    [A,B] = linSys(tNow);
    [KK,S,E] = lqr(A,B,Q,R);
    ssSoln(i).t = 0;
    ssSoln(i).K = KK;
    ssSoln(i).S = S;
    ssSoln(i).E = E;
end

ssK = reshape([ssSoln.K],2,nTime);
figure(4); clf;
subplot(2,1,1); hold on;
plot(tSol,K(1,:),'k.','MarkerSize',15)
plot(tSol,ssK(1,:),'bo','MarkerSize',10)
legend('finite','infinite')
xlim(tSpan);
ylabel('Kx')
title('Compare finite vs infinite horizon gains')
subplot(2,1,2); hold on;
plot(tSol,K(2,:),'k.','MarkerSize',15)
plot(tSol,ssK(2,:),'bo','MarkerSize',10)
legend('finite','infinite')
xlim(tSpan);
ylabel('Ky')
xlabel('t')


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Run a bunch of simulations                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

controller = @(t,z)stabilizingController(t,z,xFit,yFit,uFit,kxFit,kyFit);
userFunc = @(t,z)rhs(t,z,controller(t,z));
zNom = [xSol(1);ySol(1)];

nSim = 25;
xSim = zeros(nTime,nSim);
ySim = zeros(nTime,nSim);
del = 0.1;  %Initial position error amplitude
for i=1:nSim
    try
        z0 = zNom + del*randn(2,1);
        [~, yout] = ode45(userFunc,tSol,z0);
        xSim(:,i) = yout(:,1);
        ySim(:,i) = yout(:,2);
    catch ME
        xSim(:,i) = z0(1);
        ySim(:,i) = z0(2);
    end
end

% Plot the simulated trajectories:
figure(1); hold on;  %Plot on top of dynamics vector field
for i=1:nSim
    plot(xSim(:,i),ySim(:,i),'k-','LineWidth',1);
end



