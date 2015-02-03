function soln = cannon_multipleShooting(guess,target,param)
% soln = cannon_multipleShooting(guess,target,param)
%
% This function uses single shooting to solve the cannon problem.
%
% INPUTS:
%   guess.initSpeed
%   guess.initAngle
%   target.x
%   target.y
%   param.c = quadratic drag coefficient
%   param.nGrid = number of grid points for the integration method
%   param.nSegment = number of trajectory segments
%
% OUTPUTS:
%   soln.t
%   soln.x
%   soln.y
%   soln.dx
%   soln.dy
%

global CANNON_MULTIPLESHOOTING;  %Used for diagnostics and visualization only
CANNON_MULTIPLESHOOTING = [];

%%% Run a simulation to get the initial guess:
init.speed = guess.initSpeed;
init.angle = guess.initAngle;
traj = simulateCannon(init,param);
guess.dx0 = traj.dx(1);  %Initial horizontal speed
guess.dy0 = traj.dy(1);  %Initial vertical speed
guess.T = traj.t(end);  %Trajectory duration

%%% Break the guess trajectory at segment bounds:
guess.t = linspace(0,guess.T,param.nSegment+1); guess.t(end) = [];
guess.z = interp1(traj.t', [traj.x; traj.y; traj.dx; traj.dy]', guess.t', 'spline')';

%%% Store the initial guess for the problem:
nState = 4;  %Number of states in the problem (x,y,dx,dy)
problem.x0 = [guess.T, reshape(guess.z,1,nState*param.nSegment)];
problem.lb = [];    % Lower bound on decision variables
problem.ub = [];   % Upper bound on decision variables

%%% Set up the linear constraints (there are none);
problem.Aineq = [];  problem.Aeq = [];
problem.bineq = [];  problem.beq = [];

%%% Set up the user-defined functions:
problem.objective = @(z)objective(z(4),z(5));  %Objective (cost) function
problem.nonlcon = @(z)nonLinCst(z,target,param);   %NonLinear constraints

%%% Set up the options for the solver:
problem.solver = 'fmincon';
if param.diagnostics.enable  %Then record full diagnostics:
    problem.options = optimset(...
        'Display','iter',...
        'OutputFcn',@(z,optimVal,state)outFun(z,optimVal,state,target,param));
else %Run things quickly!
    problem.options = optimset(...
        'Display','final');
end

%Use FMINCON to solve the constrained optimization problem:
[zSoln, fVal, exitFlag] = fmincon(problem);

%Call the constraint function one final time to get the trajectory:
[~, ~, t, zTraj] = nonLinCst(zSoln,target,param);

%%% Store the trajectory in a nice format:
soln.t = t;
soln.x = zTraj(1,:);
soln.y = zTraj(2,:);
soln.dx = zTraj(3,:);
soln.dy = zTraj(4,:);
soln.success = exitFlag == 1;
soln.cost = fVal;

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                Objective function for multiple shooting                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function cost = objective(dx,dy)
% Objective function for optimization. Here we set the objective function
% to be proportional to the initial energy of the cannon ball. Note that

cost = dx.*dx + dy.*dy;

end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Non-linear constraint function for single shooting            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [C, Ceq,tTraj,zTraj] = nonLinCst(decVar,target,param)
% This is the key function!
% There are two key constraints here: Boundary Conditions & Dynamics
% The dynamics are implemented using a multiple parallel simulations
% There are constraints placed on both the initial and final states
% The simulation is implemented using 4th-order runge-kutta*.

nState = 4;  %Number of states in the problem (x,y,dx,dy)
nSegment = param.nSegment;
tEnd = decVar(1);
z0 = reshape(decVar(2:end),nState,param.nSegment);

% Run a simulation from the start of each segment, in parallel
nSub = param.nSubStep;  %Number of sub-steps for the integration method
tSim = linspace(0,tEnd/nSegment, nSub+1);
z = rk4_cannon(tSim,z0,param.c); %Simulate the trajectory
% Index Info:  z(nState,nSegment,nSubStep)

%%% Boundary Value Constraints:
BoundaryInit = [z(1,1,1); z(2,1,1)]; %Initial Position
BoundaryFinal = [z(1,end,end); z(2,end,end)]...
    - [target.x; target.y]; %Final Position

%%% Defect Constraints:
zEnd = z(:, 1:(end-1), end);  %States at the end of a segment
zStart = z(:, 2:end, 1);   %States at the beginning of a segment
Defects = reshape(zStart-zEnd,nState*(nSegment-1),1);

C = [];  %No inequality constraints
Ceq = [BoundaryInit; Defects; BoundaryFinal];  %Boundary Condition

if nargout==4
    %Then return the entire trajectory
    tTraj = zeros(1,nSegment*nSub+1);
    zTraj = zeros(nState,nSegment*nSub+1);
    
    %Stitch together the trajectory
    idx = 0;
    
    tSegment = linspace(0,tEnd,nSegment+1); tSegment(end) = [];
    for i=1:nSegment
        for j=1:nSub
            idx = idx+1;
            tTraj(idx) = tSim(j) + tSegment(i);
            zTraj(:,idx) = z(:,i,j);
        end
    end
    tTraj(end) = decVar(1);
    zTraj(:,end) = z(:,end,end);
end

end

% * I use a fixed-order method (rather than ode45) because of the improved
% consistency in the evaluation (the fixed-order method performs the exact
% same arithmetic operations on every call, where a variable order might
% not - for example, by adjusting the grid spacing, which can cause noise
% in the gradient estimates in the optimization method).





%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%             functions for diagnostics and visualization                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function stop = outFun(decVar,optimVal,state,target,param)
% This function is only used for visualization. It has nothing to do with
% the single shooting method.

global CANNON_MULTIPLESHOOTING;  %Keeping track of iteration details

stop = false;

switch state
    case 'init'
        figure(102); clf; hold on;
        CANNON_MULTIPLESHOOTING.fVal = [];
        CANNON_MULTIPLESHOOTING.iter = 0;
        maxHeight = 0.5*target.x + target.y;
        CANNON_MULTIPLESHOOTING.axis = ...  %Force the axis to be consistent
            [-0.2*target.x, 1.2*target.x,...  %horizontal limits
            -0.2*maxHeight, 1.2*maxHeight]; %Vertical limits
        
        plotTraj_multipleShooting([], target, CANNON_MULTIPLESHOOTING.iter);
        axis(CANNON_MULTIPLESHOOTING.axis);
        drawnow; pause(param.diagnostics.plotPause);
        if param.diagnostics.writeGif
            set(gcf,'Position',[100,100,param.diagnostics.gifPixelDim]);
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            imwrite(imind,cm,param.diagnostics.gifName,'gif',...
                'Loopcount',inf,...
                'DelayTime',param.diagnostics.plotPause);
        end
    case 'iter'
        nState = 4;  %Number of states in the problem (x,y,dx,dy)
        nSegment = param.nSegment;
        tEnd = decVar(1);
        z0 = reshape(decVar(2:end),nState,param.nSegment);
        
        % Run a simulation from the start of each segment, in parallel
        nSub = param.nSubStep;  %Number of sub-steps for the integration method
        tSim = linspace(0,tEnd/nSegment, nSub+1);
        z = rk4_cannon(tSim,z0,param.c); %Simulate the trajectory
        
        %%% Store details about each iteration:
        CANNON_MULTIPLESHOOTING.fVal = [CANNON_MULTIPLESHOOTING.fVal; optimVal.fval];
        CANNON_MULTIPLESHOOTING.iter = CANNON_MULTIPLESHOOTING.iter + 1;
        
        plotTraj_multipleShooting(z, target, CANNON_MULTIPLESHOOTING.iter);
        axis(CANNON_MULTIPLESHOOTING.axis);
        drawnow; pause(param.diagnostics.plotPause);
        
        if param.diagnostics.writeGif
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            imwrite(imind,cm,param.diagnostics.gifName,'gif','WriteMode','append');
        end
    case 'done'
        hold off
    otherwise
end

end


%%%% Special plotting function for use inside of FMINCON

function plotTraj_multipleShooting(z, target, iter)
% Plots a cannon ball trajectory (x,y), with speed trajectory (dx,dy)
% target.x = horizontal target
% target.y = vertical target
%

drawTraj = iter > 0;

% Plot the solution:
hold on;
if ~drawTraj  %Then draw the background
    xGround = [-0.2*target.x, 1.2*target.x];
    yGround = [0, 0];
    plot(xGround, yGround,'color',[0.5, 0.2, 0.1],'LineWidth',4);
    drawTree(0.6*target.x, 0, 1);  %Plot a tree for scale
    plot(0,0,'b.','MarkerSize',35);   %Start
    plot(target.x,target.y,'rx','LineWidth',4,'MarkerSize',14)   %Finish
else  %Draw the trajectory
    nSegment = size(z,2);
    for i=1:nSegment
        x = z(1,i,:); x = x(:);
        y = z(2,i,:); y = y(:);
        plot(x,y,'k-','LineWidth',3);  %Trajectory
    end
end

title(sprintf('Cannon Ball Trajectory:   (iteration = %2d)',iter));

axis equal; hold off;
xlabel('Horizontal Position')
ylabel('Vertical Position')

end

