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
%   param.dynamics.c = quadratic drag coefficient
%   param.singleShooting.nGrid = number of grid points for the integration method
%   param.diagnostics = struct for diagnostics 
%
% OUTPUTS:
%   soln.t
%   soln.x
%   soln.y
%   soln.dx
%   soln.dy
%

global ITER_LOG_SINGLESHOOTING;  %Used for diagnostics and visualization only
ITER_LOG_SINGLESHOOTING = [];  

%%% Run a simulation to get the initial guess:
init.speed = guess.initSpeed;
init.angle = guess.initAngle;
P.c = param.dynamics.c; 
P.nGrid = param.singleShooting.nGrid;
traj = simulateCannon(init,P);
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
problem.objective = @(decVar)objective(decVar(1),decVar(2));  %Objective (cost) function
problem.nonlcon = @(decVar)nonLinCst(decVar,target,P);   %NonLinear constraints

%%% Set up the options for the solver:
problem.solver = 'fmincon';
if param.diagnostics.enable  %Then record full diagnostics:
    problem.options = optimset(...
        'Display','iter',...
        'OutputFcn',@outFun);
else %Run things quickly!
    problem.options = optimset(...
        'Display','off');
end

%Use FMINCON to solve the constrained optimization problem:
[zSoln, fVal, exitFlag] = fmincon(problem);

%Call the constraint function one final time to get the trajectory:
[~, ~, t, zTraj] = nonLinCst(zSoln,target,P);

%%% Store the trajectory in a nice format:
soln.t = t;
soln.x = zTraj(1,:);
soln.y = zTraj(2,:);
soln.dx = zTraj(3,:);
soln.dy = zTraj(4,:);
soln.success = exitFlag == 1;
soln.cost = fVal;
soln.method = 'Single Shooting';

%%% Run diagnostics on the solution if desired:
if param.diagnostics.enable
    diagnostics_singleShooting(target,param)
else
    figure(param.diagnostics.figNum.singleShooting); clf;
    plotSoln(soln, target, param);
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Non-linear constraint function for single shooting            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [C, Ceq, t, z] = nonLinCst(decVar,target,P)
% This is the key function!
% There are two key constraints here: Boundary Conditions & Dynamics
% The dynamics are implemented using a single simulation
% The initial boundary conditions are directly enforced, and the final
% boundary conditions are enforced via the nonlinear constraint: Ceq
% The simulation is implemented using 4th-order runge-kutta*.

x0 = 0; y0 = 0;  %Trajectory starts at the origin
dx0 = decVar(1); dy0 = decVar(2);  T = decVar(3);  %Unpack decision variables

z0 = [x0;y0;dx0;dy0];  %Assemble initial dynamical state for simulation

nGrid = P.nGrid;
t = linspace(0,T,nGrid);  %Build grid in time

z = rk4_cannon(t,z0,P.c); %Simulate the trajectory

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
%                     Data-logging and Diagnostics:                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function stop = outFun(decVar,optimVal,state)
% This function is used for logging the progress of fmincon throughout the
% optimization run. It does not affect the optimization process.

global ITER_LOG_SINGLESHOOTING;  %Keeping track of iteration details

stop = false;

switch state
    case 'init'
    case 'iter'
        iter = optimVal.iteration+1;
        ITER_LOG_SINGLESHOOTING(iter).optimVal = optimVal;
        ITER_LOG_SINGLESHOOTING(iter).decVar = decVar;
    case 'done'
    otherwise
end

end

