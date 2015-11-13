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
qLow = (pi/180)*[-110; 5];
qUpp = (pi/180)*[30; 160];
dqMax = [3;3];

% Waypoints
tNode = [0, 1, 2, 3];  %time
qNode = (pi/180)*[30, -10, -60  -70;  %angle 1
    50, 90, 60, 10];    % angle 2

%Order of interpolating polynomial in each segment;
nGrid = [8,8,8];

%%%% SOLVE !! %%%%
nJoint = length(qLow);
for i=1:nJoint
    problem(i).qLow = qLow(i); %#ok<*SAGROW>
    problem(i).qUpp = qUpp(i);
    problem(i).dqMax = dqMax(i);
    problem(i).tNode = tNode;
    problem(i).qNode = qNode(i,:);
    problem(i).nGrid = nGrid;
    soln(i) = smoothJointTrajectory(problem(i));
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                             Plot Solution                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(3); clf;
nSegment = length(nGrid);
Color = getDefaultPlotColors();

for j = 1:nJoint
    
    
    % Joint constraints:
    tSpan = soln(j).grid.t([1,end]);
    
    subplot(2,nJoint,j); hold on;
    plot(tSpan,qLow(j)*[1,1],'k--','LineWidth',1);
    plot(tSpan,qUpp(j)*[1,1],'k--','LineWidth',1);
    
    subplot(2,nJoint,j+nJoint); hold on;
    plot(tSpan,dqMax(j)*[1,1],'k--','LineWidth',1);
    plot(tSpan,-dqMax(j)*[1,1],'k--','LineWidth',1);
    
    % Plot the segment boundaries
    for i=2:nSegment
        subplot(2,nJoint,j); hold on; hold on;
        plot(tNode(i)*[1,1],[qLow(j),qUpp(j)],'k-','LineWidth',1);
        subplot(2,nJoint,j+nJoint); hold on;
        plot(tNode(i)*[1,1],dqMax(j)*[-1,1],'k-','LineWidth',1);
    end
    
    % Plot the chebyshev nodes
    for i=1:nSegment
        t = soln(j).segment(i).grid.t;
        q = soln(j).segment(i).grid.q;
        dq = soln(j).segment(i).grid.dq;
        subplot(2,nJoint,j); hold on; hold on;
        plot(t,q,'ko');
        subplot(2,nJoint,j+nJoint); hold on;
        plot(t,dq,'ko');
    end
    
    % Plot the interpolated trajectories
    for i=1:nSegment
        t = soln(j).segment(i).interp.t;
        q = soln(j).segment(i).interp.q;
        dq = soln(j).segment(i).interp.dq;
        subplot(2,nJoint,j); hold on; hold on;
        plot(t,q,'Color',Color(i,:));
        subplot(2,nJoint,j+nJoint); hold on;
        h = plot(t,dq,'Color',Color(i,:));
    end
    
    % Labels
    subplot(2,nJoint,j);
    ylabel('angle (rad)')
    title(sprintf('Joint: %d',j))
    subplot(2,nJoint,j+nJoint);
    ylabel('rate (rad/s)')
    xlabel('time (s)')
    
    % Display error status:
    if soln(j).info.exitFlag ~=1
        subplot(2,nJoint,j); hold on;
        title('NO SOLUTION FOUND')
        subplot(2,nJoint,j+nJoint); hold on;
        title('NO SOLUTION FOUND')
    end
    
end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Draw Robot Arm                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

l1 = 1.0;
l2 = 0.8;

getDir = @(q)( [-sin(q);cos(q)] );  %Gets unit vector associated with orientation
getP1 = @(q1)( l1*getDir(q1) );  %Position of elbow joint
getP2 = @(q1,q2)( l1*getDir(q1) + l2*getDir(q2+q1) );  %Position of end effector

q1 = soln(1).grid.q;
q2 = soln(2).grid.q;
p1 = getP1(q1);
p2 = getP2(q1,q2);
soln(1).grid.p = p1;
soln(2).grid.p = p2;

for i = 1:nSegment
    q1 = soln(1).segment(i).interp.q;
    q2 = soln(2).segment(i).interp.q;
    p1 = getP1(q1);
    p2 = getP2(q1,q2);
    soln(1).segment(i).interp.p = p1;
    soln(2).segment(i).interp.p = p2;
end


%%%% Draw a figure showing the robot arm and each key-frame
figure(4); clf; hold on;
for i=0:nSegment
    if i==0
        xArm = [0, soln(1).segment(1).interp.p(1,1), soln(2).segment(1).interp.p(1,1)];
        yArm = [0, soln(1).segment(1).interp.p(2,1), soln(2).segment(1).interp.p(2,1)];
    else
        xArm = [0, soln(1).segment(i).interp.p(1,end), soln(2).segment(i).interp.p(1,end)];
        yArm = [0, soln(1).segment(i).interp.p(2,end), soln(2).segment(i).interp.p(2,end)];
    end
    %%% Plot the arm
    plot(xArm, yArm, 'ko','LineWidth',2,'MarkerSize',10);
    plot(xArm, yArm, 'k-','LineWidth',4);
    
    %%% Plot the trajectory
    if i~=0
        p1 = soln(1).segment(i).interp.p;
        p2 = soln(2).segment(i).interp.p;
        plot(p1(1,:),p1(2,:), '--', 'Color', Color(i,:));
        plot(p2(1,:),p2(2,:), '--', 'Color', Color(i,:));
    end
end


axis((l1+l2)*[-1,1,0,1]); axis equal; axis off;

