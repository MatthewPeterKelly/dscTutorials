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
qLow = -2.5; problem.qLow = qLow;  
qUpp = 1.5; problem.qUpp = qUpp;
dqMax = 5; problem.dqMax = dqMax;   %Joint speed limit

% Waypoints
tNode = [0, 1.3, 2, 2.7, 3.0]; problem.tNode = tNode;   %time
qNode = [0, 1,  -2, -0.2, -1]; problem.qNode = qNode;   %angle

%Order of interpolating polynomial in each segment;
problem.nGrid = [13, 12, 14, 7];  

%%%% SOLVE !! %%%%
soln = smoothJointTrajectory(problem);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Plot Solution                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(3); clf;
nSegment = length(problem.nGrid);

% Joint constraints:
tSpan = soln.grid.t([1,end]);
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
    t = soln.segment(i).grid.t;
    q = soln.segment(i).grid.q;
    dq = soln.segment(i).grid.dq;
    subplot(2,1,1); hold on;
    plot(t,q,'ko');
    subplot(2,1,2); hold on;
    plot(t,dq,'ko');
end

% Plot the interpolated trajectories
for i=1:nSegment
        t = soln.segment(i).interp.t;
    q = soln.segment(i).interp.q;
    dq = soln.segment(i).interp.dq;
    subplot(2,1,1); hold on;
    plot(t,q);
    subplot(2,1,2); hold on;
    plot(t,dq);
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

