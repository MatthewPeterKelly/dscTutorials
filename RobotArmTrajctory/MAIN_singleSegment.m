% MAIN.m -- Robot Arm Trajectory
%
% Given an initial and final state, as well as joint angle and rate
% constraints, find the minimum-jerk trajectory that solves the boundary
% value problem.
%
%

n = 7;  %Order of interpolating polynomial;

% Problem specifications:
tStart = 0;  
tFinal = 1.0;
qLow = -pi;
qUpp = pi;
dqMax = 1.5;   %Joint speed limit

% Boundary values
qStart = 0;
qFinal = 1;
dqStart = 0;
dqFinal = -1.5;

%%%% Boundary Values:
Aeq = zeros(4,n); beq = zeros(4,1);
Aeq(1,1) = 1; beq(1) = qStart;
Aeq(2,end) = 1; beq(2) = qFinal;
Aeq(3:4,:) = D([1,end],:); beq(3:4) = [dqStart; dqFinal];

%%%% Rate constraints and cost for a single segment:
d = [tStart, tFinal];
[A,b,H,t] = chebyshevSegment(n,d,dqMax);

%%%% Options:
options = optimset(...
    'Display','iter',...  % {'iter','final'}
    'Algorithm','interior-point-convex');

%%%% Build Problem:
problem.H = H;
problem.f = zeros(n,1);
problem.Aineq = A;
problem.bineq = b;
problem.Aeq = Aeq;
problem.beq = beq;
problem.lb = LB;
problem.ub = UB;
problem.x0 = [];
problem.options = options;
problem.solver = 'quadprog';

%%%% Solve!
[x, fVal, exitFlag, output] = quadprog(problem);

% Plot the solution:
tt = linspace(t(1),t(end),100);
[xx, dxx] = chebyshevInterpolate(x',tt,[t(1),t(end)]);
figure(2); clf; 
subplot(2,1,1); hold on;
plot(tt,xx);
plot(t,x,'ko');
subplot(2,1,2); hold on;
plot(tt,dxx);
plot(t,D*x,'ko');




