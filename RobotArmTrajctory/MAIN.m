% MAIN.m -- Robot Arm Trajectory
%
% Given an initial and final state, as well as joint angle and rate
% constraints, find the minimum-jerk trajectory that solves the boundary
% value problem.
%
%

n = 15;  %Order of interpolating polynomial;

% Problem specifications:
tStart = 0;  
tFinal = 1;
qLow = -pi;
qUpp = pi;
dqMax = 1.5;   %Joint speed limit

% Boundary values
qStart = 0;
qFinal = 1;
dqStart = 0;
dqFinal = -1.5;

% Derivatives
D = chebyshevDifferentiationMatrix(n,[tStart,tFinal]);   %Rates
DD = D*D;    %Accelerations
DDD = DD*D;   %Jerk

% Chebyshev grid and quadrature weights:
[t,w] = chebyshevPoints(n,[tStart,tFinal]);

%%%% Quadratic Program:
%   min 0.5*x'*H*x + f'*x   subject to:  A*x <= b
%    x                                   Aeq*x == beq
%                                        LB <= x <=UB

%%%% Boundary Values:
Aeq = zeros(4,n); beq = zeros(4,1);
Aeq(1,1) = 1; beq(1) = qStart;
Aeq(2,end) = 1; beq(2) = qFinal;
Aeq(3:4,:) = D([1,end],:); beq(3:4) = [dqStart; dqFinal];

%%%% Dynamics:
% u == DDD*x;   %Equality Constraint - expressed directly in cost function

%%%% Joint Rate Limits:
% -dqMax <= D*x <= dqMax
A = [D;-D];
b = [dqMax*ones(n,1); dqMax*ones(n,1)];

%%%% Joint Angle Limits:
% qLow <= x <= qUpp
LB = qLow*ones(n,1);
UB = qUpp*ones(n,1);

%%%% Objective Function:
% min  w*(u^2) = x'*(DDD'*W*DDD);  W = diag(w);
W = diag(w);
H = 0.5*(DDD'*W*DDD);

%%%% Initialize:    (Linear trajectory)
xGuess = interp1([tStart;tFinal],[qStart;qFinal],t');

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
problem.x0 = xGuess;
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




