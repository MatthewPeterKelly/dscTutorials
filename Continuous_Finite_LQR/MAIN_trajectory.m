%MAIN
%
% This script is used for testing the continuous, finite horizon, LQR
% solver that I'm working on.
%
%
clear; clc;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       User Set Parameters                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nSim = 25;
testPerturbation = 0.1;  %Initial position error amplitude

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Draw the dynamical system                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

n = 13;
xLim = [-4,1.5];
yLim = [-4,1];
x = linspace(xLim(1),xLim(2),n);
y = linspace(yLim(1),yLim(2),n);

[xx,yy] = ndgrid(x,y);

[dxx, dyy] = dynamics(xx,yy,zeros(n,n));

figure(1); clf; hold on;
title('Dynamical System - Simulations');
quiver(xx,yy,dxx,dyy);
xlabel('x');
ylabel('y');


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Compute a reference trajectory                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

options = odeset();
options.Events = @(t,z)eventFunc(t,z,xLim,yLim);
tSpan = [0,-10];

control = @(t) -2*sin(0.4*t-0.1)+0.6*cos(0.9*t+0.1);

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
axis(1.2*[xLim,yLim]); axis equal;

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
    [KK,SS,EE] = lqr(A,B,Q,R);
    ssSoln(i).t = 0;
    ssSoln(i).K = KK;
    ssSoln(i).S = SS;
    ssSoln(i).E = EE;
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
%                     Lyapunov Stability Analysis                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Let V(x) = x'Sx, where S is the cost-to-go matrix from LQR solution:

%%%% TODO %%%%

% Plan:
%
%   1) Represent the S matrix, which is time-varying, as a polynomial.
%
%   2) Compute the derivative of S wrt time.
%
%   3) Try the Lyapunov function:
%           V(x,t) = x'*S(t)*x     % x = perturbation from nominal
%           dV(x,t) = 2*x*f(x,t)*S(t)*dS(t)   %Chain Rule  (check math...)
%   
%   4) Verify stability by checking signs of lyapunov function and
%      derivative?
%  
%   5) Not sure if this whole approach is correct - check against papers...
%


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Run a bunch of simulations                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

controller = @(t,z)stabilizingController(t,z(1,:),z(2,:),xFit,yFit,uFit,kxFit,kyFit);
userFunc = @(t,z)rhs(t,z,controller(t,z));
zNom = [xSol(1);ySol(1)];

xSim = zeros(nTime,nSim);
ySim = zeros(nTime,nSim);
uSim = zeros(nTime,nSim);
failFlag = false(nSim,1);
for i=1:nSim
    try
        z0 = zNom + testPerturbation*randn(2,1);
        [~, yout] = ode45(userFunc,tSol,z0);
        xSim(:,i) = yout(:,1);
        ySim(:,i) = yout(:,2);
        uSim(:,i) = controller(tSol,yout');
    catch ME
        xSim(:,i) = z0(1);
        ySim(:,i) = z0(2);
        uSim(:,i) = 0;
        failFlag(i) = true;
    end
end

% Plot the simulated trajectories:
figure(1); hold on;  %Plot on top of dynamics vector field
for i=1:nSim
    plot(xSim(:,i),ySim(:,i),'k-','LineWidth',1);
end

%%%% Plot trajectories against time:
figure(5); clf;

for i=1:nSim
    if ~failFlag(i)
        subplot(3,1,1); hold on;
        plot(tSol,xSim(:,i),'k-','LineWidth',1)
        subplot(3,1,2); hold on;
        plot(tSol,ySim(:,i),'k-','LineWidth',1)
        subplot(3,1,3); hold on;
        plot(tSol,uSim(:,i),'k-','LineWidth',1)
    end
end

subplot(3,1,1); hold on;
plot(tSol,xSol,'r-','LineWidth',3)
xlim(tSpan);
ylabel('x')
title('Compare simulations against reference trajectory')
subplot(3,1,2); hold on;
plot(tSol,ySol,'r-','LineWidth',3)
xlim(tSpan);
ylabel('y')
subplot(3,1,3); hold on;
plot(tSol,uSol,'r-','LineWidth',3)
xlim(tSpan);
ylabel('u')
xlabel('t')


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%    Computate the backward reachable set   --  Convex Hull Method        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nSamples = 500;  %How many simulations to run?
nPointsPerSample = 10;  %Used for computing reachable set

% What is "close enough" to the final state?
xRadFinal = 0.015;
yRadFinal = 0.015;

%Compute a random set of initial conditions
IC = randDisk(nSamples,[xRadFinal,yRadFinal]);

%Simulation runs backwards in time
tSpanFlip = linspace(tSpan(2),tSpan(1),nPointsPerSample);   %Run system backwards in time

%Create a data structure for storing the point cloud:
ReachCvx.t = ones(nSamples,1)*tSpanFlip;  % Time
ReachCvx.x = zeros(nSamples,nPointsPerSample);  % x position
ReachCvx.y = zeros(nSamples,nPointsPerSample);  % y position
ReachCvx.c(nPointsPerSample).x = []; %x contour
ReachCvx.c(nPointsPerSample).y = []; %x contour

%Run the simulation:
for i=1:nSamples
    idxLow = (i-1)*nPointsPerSample+1;
    idxUpp= i*nPointsPerSample;
    [~, yout] = ode45(userFunc,tSpanFlip,IC(:,i));
    ReachCvx.x(i,:) = yout(:,1);
    ReachCvx.y(i,:) = yout(:,2);
end

%Compute the convex hull at each time step
for i=1:nPointsPerSample
    k = convhull(ReachCvx.x(:,i),ReachCvx.y(:,i));
    ReachCvx.c(i).t = ReachCvx.t(k,i);
    ReachCvx.c(i).x = ReachCvx.x(k,i);
    ReachCvx.c(i).y = ReachCvx.y(k,i);
end


%Plot the reachable sets against time:
figure(6); clf; hold on;
colors = jet(nPointsPerSample);
for i=1:nPointsPerSample
    %Convex Hull
    plot3(ReachCvx.c(i).x, ReachCvx.c(i).y, ReachCvx.c(i).t,'color',colors(i,:));
    %Point Cloud:
    plot3(ReachCvx.x(:,i), ReachCvx.y(:,i), ReachCvx.t(:,i),'.','MarkerSize',10,'color',colors(i,:));
end

%plot the nominal trajectory
plot3(xSol,ySol,tSol,'k-','LineWidth',3);



title('Convex Hull approximation of the reachable set');
xlabel('x'); ylabel('y'); zlabel('time');
view(3);



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%    Computate the backward reachable set   --  Front Propagation         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nSamples = 250;  %How many simulations to run?
nPointsPerSample = 10;  %Used for computing reachable set

% What is "close enough" to the final state?
xRadFinal = 0.015;
yRadFinal = 0.015;

%Compute initial conditions on the circle around the final set:
angle = linspace(0,2*pi,nSamples+1); angle(end) = [];
IC = [cos(angle); sin(angle)];
IC(1,:) = IC(1,:)*xRadFinal;
IC(2,:) = IC(2,:)*yRadFinal;

%Simulation runs backwards in time
tSpanFlip = linspace(tSpan(2),tSpan(1),nPointsPerSample);   %Run system backwards in time

%Create a data structure for storing the point cloud:
ReachFrt.t = ones(nSamples,1)*tSpanFlip;  % Time
ReachFrt.x = zeros(nSamples,nPointsPerSample);  % x position
ReachFrt.y = zeros(nSamples,nPointsPerSample);  % y position

%Run the simulation:
for i=1:nSamples
    idxLow = (i-1)*nPointsPerSample+1;
    idxUpp= i*nPointsPerSample;
    [~, yout] = ode45(userFunc,tSpanFlip,IC(:,i));
    ReachFrt.x(i,:) = yout(:,1);
    ReachFrt.y(i,:) = yout(:,2);
end

%Plot the reachable sets against time:
figure(7); clf; hold on;
colors = jet(nPointsPerSample);
idx = [1:size(ReachFrt.x,1),1];  %Make the contours wrap around
for i=1:nPointsPerSample
    %Front contour
    plot3(ReachFrt.x(idx,i), ReachFrt.y(idx,i), ReachFrt.t(idx,i),'color',colors(i,:));
    %Point Cloud:
    plot3(ReachFrt.x(:,i), ReachFrt.y(:,i), ReachFrt.t(:,i),'k.','MarkerSize',15);
    %Fill the central region:
    h = patch(ReachFrt.x(idx,i), ReachFrt.y(idx,i), ReachFrt.t(idx,i),colors(i,:));
end
plot3(xSol,ySol,tSol,'k-');

hp = findobj(gcf,'type','patch');
set(hp,'facealpha',0.7)
set(hp,'edgealpha',0.1)

title('Front Propagation approximation of the reachable set');
xlabel('x'); ylabel('y'); zlabel('time');
view(3);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Computation with the Level Set Toolbox                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%This section of code requires the levelSetToolbox to operate:
%         http://www.cs.ubc.ca/~mitchell/ToolboxLS/
if exist('odeCFLset','file')
    %This section of code is still under development. As of 11/20/2014 the
    %following two scripts should run, but they are slow. It also seems
    %that there is some numerical dissapation, that is causing inaccurate
    %results for any reasonable level of discretization.
    
    %%%% reachableForward
    %%%% reachableForwardRel
end





