% MAIN.m -- Robot Arm Trajectory
%
% Given an initial and final state, as well as joint angle and rate
% constraints, find the minimum-jerk trajectory that passes through all
% node points and has an initial and final velocity of zero.
%
%

% NOTE - Code is still in progress, needs to be cleaned up.


n = [15,11,9];  %Order of interpolating polynomial in each segment;
N = sum(n);

% Problem specifications:
qLow = -pi;
qUpp = pi;
dqMax = 5.0;   %Joint speed limit

% Boundary values
tNode = [0, 1.3, 2, 2.5];
qNode = [0,1,-2,0];

% Rate and cost along each segment:
[A1,b1,H1,t1,D1] = chebyshevSegment(n(1),tNode(1:2),dqMax);
[A2,b2,H2,t2,D2] = chebyshevSegment(n(2),tNode(2:3),dqMax);
[A3,b3,H3,t3,D3] = chebyshevSegment(n(3),tNode(3:4),dqMax);
A = blkdiag(A1,A2,A3);
b = [b1;b2;b3];
H = blkdiag(H1,H2,H3);
D = blkdiag(D1,D2,D3);
t = [t1, t2,t3];

%%%% Boundary Values:
Aeq = zeros(10,N); beq = zeros(10,1);
finalIdx = cumsum(n);
startIdx = 1 + [0, finalIdx(1:(end-1))];

Aeq(1,startIdx(1)) = 1;  beq(1) = qNode(1);
Aeq(2,startIdx(2)) = 1;  beq(2) = qNode(2);
Aeq(3,startIdx(3)) = 1;  beq(3) = qNode(3);

Aeq(4,finalIdx(1)) = 1;  beq(4) = qNode(2);
Aeq(5,finalIdx(2)) = 1;  beq(5) = qNode(3);
Aeq(6,finalIdx(3)) = 1;  beq(6) = qNode(4);

Aeq(7,:) = D(1,:);  %zero initial velocity
Aeq(8,:) = D(end,:); %zero initial velocity

Aeq(9,:) = D(startIdx(2),:) - D(finalIdx(1),:); % Zero rate defect between segments
Aeq(10,:) = D(startIdx(3),:) - D(finalIdx(2),:);  % Zero rate defect between segments

%%%% Options:
options = optimset(...
    'Display','iter',...  % {'iter','final'}
    'Algorithm','interior-point-convex');

%%%% Build Problem:
problem.H = H;
problem.f = zeros(N,1);
problem.Aineq = A;
problem.bineq = b;
problem.Aeq = Aeq;
problem.beq = beq;
problem.lb = qLow*ones(N,1);
problem.ub = qUpp*ones(N,1);
problem.x0 = [];
problem.options = options;
problem.solver = 'quadprog';

%%%% Solve!
[x, fVal, exitFlag, output] = quadprog(problem);

% Plot the solution:
figure(3); clf;
subplot(2,1,1); hold on;
plot(t,x);
plot(tNode, qNode,'ko');




