% MAIN.m -- Robot Arm Trajectory
%
% Given an initial and final state, as well as joint angle and rate
% constraints, find the minimum-jerk trajectory that passes through all
% node points and has an initial and final velocity of zero.
%
%
clear; clc;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Problem specifications                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Joint limits
qLow = -2.5;  
qUpp = 1.5;
dqMax = 5;   %Joint speed limit

% Waypoints
tNode = [0, 1.3, 2, 2.7, 3.0];   %time
qNode = [0, 1,  -2, -0.2, -1];   %angle

%Order of interpolating polynomial in each segment;
nGrid = [13, 12, 14, 7];  

 %Second-order continuous?
flagSmooth = true; 


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Build problem matricies                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


nDecVar = sum(nGrid);  %Number of decision variables
nSegment = length(nGrid);


% Rate cosntraint and objective matrix along each segment:
for i=1:length(nGrid)
    S(i) = chebyshevSegment(nGrid(i),tNode([i,i+1]),dqMax); %#ok<SAGROW>
end
A = structBlkDiag(S,'A');
H = structBlkDiag(S,'H');
D = structBlkDiag(S,'D');
DD = structBlkDiag(S,'DD');

b = []; t = [];  % Slow memory allocation - ok since few iterations
for i=1:length(nGrid)
    b = [b; S(i).b];  %#ok<AGROW>
    t = [t; S(i).t];  %#ok<AGROW>
end

%%%% Boundary Values:
if flagSmooth    %#ok<*UNRCH>
    nCstBc = 2*nSegment + 2*(nSegment+1);  %angle + rate
else
    nCstBc = 2*nSegment + (nSegment+1);  %angle + rate
end
Aeq = zeros(nCstBc,nDecVar); beq = zeros(nCstBc,1);
finalIdx = cumsum(nGrid);
startIdx = 1 + [0, finalIdx(1:(end-1))];

cstIdx = 0;
for i=1:nSegment   %Angle at start of segment
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,startIdx(i)) = 1;  beq(cstIdx) = qNode(i);
end
for i=1:nSegment   %Angle at end of segment
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,finalIdx(i)) = 1;  beq(cstIdx) = qNode(i+1);
end

cstIdx = cstIdx + 1; Aeq(cstIdx,:) = D(1,:);  %zero initial velocity
cstIdx = cstIdx + 1; Aeq(cstIdx,:) = D(end,:);  %zero final velocity

% Defect constraint on rate at segment boundaries
for i=1:(nSegment-1)
    cstIdx = cstIdx + 1;
    Aeq(cstIdx,:) = D(startIdx(i+1),:) - D(finalIdx(i),:);
end

if flagSmooth  % Defect constraint on accel at segment boundaries
    for i=1:(nSegment-1)
        cstIdx = cstIdx + 1;
        Aeq(cstIdx,:) = DD(startIdx(i+1),:) - DD(finalIdx(i),:);
    end
end

%%%% Options:
options = optimset(...
    'Display','iter',...  % {'iter','final'}
    'Algorithm','interior-point-convex');

%%%% Build Problem:
problem.H = (H+H')/2;   %Correct for numerically introduced asymmetry
problem.f = zeros(nDecVar,1);
problem.Aineq = A;
problem.bineq = b;
problem.Aeq = Aeq;
problem.beq = beq;
problem.lb = qLow*ones(nDecVar,1);
problem.ub = qUpp*ones(nDecVar,1);
problem.x0 = [];
problem.options = options;
problem.solver = 'quadprog';


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Solve Problem                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

[x, fVal, exitFlag, output] = quadprog(problem);
for i=1:nSegment
    S(i).q = x(startIdx(i):finalIdx(i));
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Plot Solution                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(3); clf;

% Joint constraints:
tSpan = [S(1).t(1), S(end).t(end)];
subplot(2,1,1); hold on;
plot(tSpan,qLow*[1,1],'k--','LineWidth',1);
plot(tSpan,qUpp*[1,1],'k--','LineWidth',1);
subplot(2,1,2); hold on;
plot(tSpan,dqMax*[1,1],'k--','LineWidth',1);
plot(tSpan,-dqMax*[1,1],'k--','LineWidth',1);

% Plot the segment boundaries
for i=2:nSegment
    subplot(2,1,1); hold on;
    plot(tNode(i)*[1,1],[qLow,qUpp],'k-','LineWidth',1);
    subplot(2,1,2); hold on;
    plot(tNode(i)*[1,1],dqMax*[-1,1],'k-','LineWidth',1);
end
    
% Plot the chebyshev nodes
for i=1:nSegment
    subplot(2,1,1); hold on;
    plot(S(i).t,S(i).q,'ko');
    subplot(2,1,2); hold on;
    plot(S(i).t,S(i).D*S(i).q,'ko');
end

% Plot the interpolated trajectories
for i=1:nSegment
    domain = [S(i).t(1),S(i).t(end)];
    tt = linspace(domain(1),domain(2),10*nGrid(i));
    [xx, dxx] = chebyshevInterpolate(S(i).q',tt,domain);
    subplot(2,1,1); hold on;
    plot(tt,xx);
    subplot(2,1,2); hold on;
    plot(tt,dxx);
end

% Labels
subplot(2,1,1);
ylabel('angle (rad)')
title('optimal trajectory')
subplot(2,1,2);
ylabel('rate (rad/s)')
xlabel('time (s)')

% Display error status:
if exitFlag ~=1
   subplot(2,1,1); hold on;
   title('NO SOLUTION FOUND')
   subplot(2,1,2); hold on;
   title('NO SOLUTION FOUND')
end

