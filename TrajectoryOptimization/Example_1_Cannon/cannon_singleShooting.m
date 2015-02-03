function soln = cannon_singleShooting(guess,target,param)
% soln = cannon_singleShooting(guess,target,param)
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
%
% OUTPUTS:
%   soln.t
%   soln.x
%   soln.y
%   soln.dx
%   soln.dy
%

global CANNON_SINGLESHOOTING;  %Used for diagnostics and visualization only
CANNON_SINGLESHOOTING = [];  

%%% Run a simulation to get the initial guess:
init.speed = guess.initSpeed;
init.angle = guess.initAngle;
traj = simulateCannon(init,param);
guess.dx0 = traj.dx(1);  %Initial horizontal speed
guess.dy0 = traj.dy(1);  %Initial vertical speed
guess.T = traj.t(end);  %Trajectory duration

%%% Set up the decision variables and bounds:
problem.x0 = [guess.dx0; guess.dy0; guess.T];
problem.lb = [-10, 0, 0];    % Lower bound on decision variables
problem.ub = [10, 10, 10];   % Upper bound on decision variables

%%% Set up the linear constraints (there are none);
problem.Aineq = [];  problem.Aeq = [];
problem.bineq = [];  problem.beq = [];

%%% Set up the user-defined functions:
problem.objective = @objective;  %Objective (cost) function
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
%                Objective function for single shooting                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function cost = objective(z)
% Objective function for optimization. Here we set the objective function
% to be proportional to the initial energy of the cannon ball. Note that

xSpeed = z(1);
ySpeed = z(2);
cost = xSpeed.*xSpeed + ySpeed*ySpeed;

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Non-linear constraint function for single shooting            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [C, Ceq, t, z] = nonLinCst(z,target,param)
% This is the key function!
% There are two key constraints here: Boundary Conditions & Dynamics
% The dynamics are implemented using a single simulation
% The initial boundary conditions are directly enforced, and the final
% boundary conditions are enforced via the nonlinear constraint: Ceq
% The simulation is implemented using 4th-order runge-kutta*.

x0 = 0; y0 = 0;  %Trajectory starts at the origin
dx0 = z(1); dy0 = z(2);  T = z(3);  %Unpack decision variables

z0 = [x0;y0;dx0;dy0];  %Assemble initial dynamical state for simulation

nGrid = param.nGrid;
t = linspace(0,T,nGrid);  %Build grid in time

z = rk4_cannon(t,z0,param.c); %Simulate the trajectory

xFinal = z(1,end);
yFinal = z(2,end);

C = [];  %No inequality constraints
Ceq = [xFinal - target.x; yFinal - target.y];  %Boundary Condition

end

% * I use a fixed-order method (rather than ode45) because of the improved
% consistency in the evaluation (the fixed-order method performs the exact
% same arithmetic operations on every call, where a variable order might
% not - for example, by adjusting the grid spacing, which can cause noise
% in the gradient estimates in the optimization method).





%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%             functions for diagnostics and visualization                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function stop = outFun(z,optimVal,state,target,param)
% This function is only used for visualization. It has nothing to do with
% the single shooting method.

global CANNON_SINGLESHOOTING;  %Keeping track of iteration details

stop = false;

x0 = 0; y0 = 0;  %Trajectory starts at the origin
dx0 = z(1); dy0 = z(2);  T = z(3);  %Unpack decision variables
switch state
    case 'init'
        figure(100); clf; hold on;
        CANNON_SINGLESHOOTING.fVal = [];
        CANNON_SINGLESHOOTING.iter = 0;
        maxHeight = 0.5*target.x + target.y;
        CANNON_SINGLESHOOTING.axis = ...  %Force the axis to be consistent
            [-0.2*target.x, 1.2*target.x,...  %horizontal limits
            -0.2*maxHeight, 1.2*maxHeight]; %Vertical limits
        
        plotTraj_singleShooting([], target, CANNON_SINGLESHOOTING.iter);
        axis(CANNON_SINGLESHOOTING.axis);
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
        z0 = [x0;y0;dx0;dy0];  %Assemble dynamic state for simulation
        
        nGrid = param.nGrid;
        time = linspace(0,T,nGrid);  %Build grid in time
        
        z = rk4_cannon(time,z0,param.c); %Simulate the trajectory
        
        %%% Store details about each iteration:
        CANNON_SINGLESHOOTING.fVal = [CANNON_SINGLESHOOTING.fVal; optimVal.fval];
        CANNON_SINGLESHOOTING.iter = CANNON_SINGLESHOOTING.iter + 1;
        
        plotTraj_singleShooting(z, target, CANNON_SINGLESHOOTING.iter);
        axis(CANNON_SINGLESHOOTING.axis);
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

function plotTraj_singleShooting(traj, target, iter)
% Plots a cannon ball trajectory (x,y), with speed trajectory (dx,dy)
% target.x = horizontal target
% target.y = vertical target
%

drawTraj = iter > 0;   

if drawTraj
    x = traj(1,:);
    y = traj(2,:);
end

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
    plot(x,y,'k-','LineWidth',3);  %Trajectory
end

title(sprintf('Cannon Ball Trajectory:   (iteration = %2d)',iter));

axis equal; hold off;
xlabel('Horizontal Position')
ylabel('Vertical Position')

end

