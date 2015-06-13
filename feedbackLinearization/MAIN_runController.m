% MAIN_passiveSimulate.m
%
% This script runs a simulation to generate a passive reference trajectory,
% and then uses feedback linearization to track the trajectory.
%

clc; clear;

param.m1 = 5;
param.m2 = 1;
param.g = 5;
param.l1 = 2;
param.l2 = 2;

% Initial state:
q1 = (pi/180)*240;
q2 = -(pi/180)*60;
dq1 = -(pi/180)*170;
dq2 = (pi/180)*190;
z0 = [q1;q2;dq1;dq2];  %Pack up initial state

tSpan = [0,0.8];  %time span for the simulation
dynFun = @(t,z)( acrobotDynamics(z,0,param) );  %passive dynamics function

% Run simulation:
sol = ode45(dynFun,tSpan,z0);
t = linspace(tSpan(1),tSpan(2),100);
z = deval(sol,t);
u = zeros(size(t));

% Animate the results:
A.plotFunc = @(t,z)( drawAcrobot(t,z,param) );
A.speed = 0.2;
A.figNum = 1;
% animate(t,z,A)

% Plot the results:
figure(2); clf; plotAcrobot(t,z,u,param)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Set up reference trajectory                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

ref.nGrid = 25;   %Number of grid points to store trajectory on
ref.c = [1 0];   %phase variable = link one angle = c*q = [1 0]*[q1;q2]
ref.H = [-1 1];  % Measurement variable = relative angle between links = H*q

tRef = linspace(tSpan(1),tSpan(2),ref.nGrid);
zRef = deval(sol,tRef);
dzRef = zeros(size(zRef));
for i=1:ref.nGrid
    dzRef(:,i) = acrobotDynamics(zRef(:,i),0,param);
end
qRef = zRef(1:2,:);
dqRef = zRef(3:4,:);
ddqRef = dzRef(3:4,:);

% Trajectories: measurement and phase vs time:
hRef = ref.H*qRef;   % Target measurement
dhRef = ref.H*dqRef;
ddhRef = ref.H*ddqRef;
pRef = ref.c*qRef; % Phase
dpRef = ref.c*dqRef;
ddpRef = ref.c*ddqRef;

% Compute derivatives wrt phase using chain rule:
dhRefdp = dhRef./dpRef;
ddhRefddp = (ddhRef - dhRefdp.*ddpRef)./(dpRef.^2);

% Represent trajectories as piecewise-polynomial
ref.pp.h = pchip(pRef,hRef);
ref.pp.dh = pchip(pRef,dhRefdp);
ref.pp.dhdt = pchip(pRef,dhRef);
ref.pp.ddh = pchip(pRef,ddhRefddp);


% Check math using finite differences:
check1.p = ref.c*z(1:2,:);
check1.h = ref.H*z(1:2,:);
check1.dh = diff(check1.h)./diff(check1.p);
check1.ddh = diff(check1.dh)./diff(check1.p(1:(end-1)));

% And compare to interpolated trajectory:
check2.p = check1.p;
check2.h = ppval(ref.pp.h, check2.p);
check2.dh = ppval(ref.pp.dh, check2.p);
check2.ddh = ppval(ref.pp.ddh, check2.p);


% Plot the interpolated trajectories against data and finite differences:
figure(3); clf;
subplot(3,1,1); hold on;
plot(check1.p, check1.h, 'r.','MarkerSize',8);
plot(check2.p, check2.h, 'k-','LineWidth',1);
ylabel('h');
xlabel('p');
legend('trajectory data','pchip approximation')
subplot(3,1,2); hold on;
plot(check1.p(1:(end-1)), check1.dh, 'r.','MarkerSize',8);
plot(check2.p, check2.dh, 'k-','LineWidth',1);
ylabel('dh');
xlabel('p');
subplot(3,1,3); hold on;
plot(check1.p(1:(end-2)), check1.ddh, 'r.','MarkerSize',8);
plot(check2.p, check2.ddh, 'k-','LineWidth',1);
ylabel('ddh');
xlabel('p');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Run a controlled simulation:                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% TRACKING GAINS:
ref.wn = 50;   %Tracking frequency
ref.xi = 0.5;  %Tracking damping ratio

ctrlFun = @(z)( controller(z,param,ref) );  %Control
sysFun = @(t,z)( acrobotDynamics(z,ctrlFun(z),param) );  % Dynamics

% z0 = z0 + 0.01*(1-2*rand(4,1));   % Add some error to make things interesting
z0 = z0 + [0;0.05;0.2;0];   %Add deterministic error for repeatability

% Run simulation:
solCtrl = ode45(sysFun,tSpan,z0);
tCtrl = linspace(tSpan(1),tSpan(2),100);
zCtrl = deval(solCtrl,tCtrl);
uCtrl = zeros(1,length(tCtrl));
for i=1:length(tCtrl)
    uCtrl(i) = ctrlFun(zCtrl(:,i));
end


figure(4); clf; plotAcrobot(tCtrl,zCtrl,uCtrl,param);

pCtrl = ref.c*zCtrl(1:2,:);
hCtrl = ref.H*zCtrl(1:2,:);
dhCtrl = ref.H*zCtrl(3:4,:);

% Plot the results:
figure(5); clf;
subplot(3,1,1); hold on;
plot(check2.p, check2.h, 'k.','MarkerSize',10);
plot(pCtrl, hCtrl, 'b-');
xlabel('p')
ylabel('h')
legend('ref','sim')
subplot(3,1,2); hold on;
plot(check2.p, ppval(ref.pp.dhdt,check2.p), 'k.','MarkerSize',10);
plot(pCtrl, dhCtrl, 'b-');
xlabel('p')
ylabel('dh')
subplot(3,1,3); hold on;
plot(pCtrl, uCtrl, 'b-');
xlabel('p')
ylabel('u')
