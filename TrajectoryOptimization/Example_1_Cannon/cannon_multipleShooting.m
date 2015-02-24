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

global ITER_LOG_MULTIPLESHOOTING;  %Used for diagnostics and visualization only
ITER_LOG_MULTIPLESHOOTING = [];

P.nSegment = param.multipleShooting.nSegment;
P.nSubStep = param.multipleShooting.nSubStep;
P.nGrid = P.nSegment*P.nSubStep;
P.c = param.dynamics.c;

%%% Run a simulation to get the initial guess:
init.speed = guess.initSpeed;
init.angle = guess.initAngle;
traj = simulateCannon(init,P);
guess.T = traj.t(end);  %Trajectory duration

%%% Break the guess trajectory at segment bounds:
guess.t = linspace(0,guess.T,P.nSegment+1); guess.t(end) = [];
guess.z = interp1(traj.t', [traj.x; traj.y; traj.dx; traj.dy]', guess.t', 'spline')';

%%% Store the initial guess for the problem:
nState = 4;  %Number of states in the problem (x,y,dx,dy)
problem.x0 = [guess.T, reshape(guess.z,1,nState*P.nSegment)];
problem.lb = [];    % Lower bound on decision variables
problem.ub = [];   % Upper bound on decision variables

%%% Set up the linear constraints (there are none);
problem.Aineq = [];  problem.Aeq = [];
problem.bineq = [];  problem.beq = [];

%%% Set up the user-defined functions:
problem.objective = @(decVar)objective(decVar(4),decVar(5));  %Objective (cost) function
problem.nonlcon = @(decVar)nonLinCst(decVar,target,P);   %NonLinear constraints

%%% Set up the options for the solver:
problem.solver = 'fmincon';
if param.diagnostics.enable  %Then record full diagnostics:
    problem.options = optimset(...
        'Display','iter',...
        'OutputFcn',@(decVar,optimVal,state)outFun(decVar,optimVal,state));
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
soln.method = 'Multiple Shooting';

%%% Run diagnostics on the solution if desired:
if param.diagnostics.enable
    diagnostics_multipleShooting(target,param)
else
    figure(param.diagnostics.figNum.multipleShooting); clf;
    plotSoln(soln, target, param);
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Non-linear constraint function for multiple shooting          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function [C, Ceq,tTraj,zTraj] = nonLinCst(decVar,target,P)
% This is the key function!
% There are two key constraints here: Boundary Conditions & Dynamics
% The dynamics are implemented using a multiple simulations
% The decision variables are time, followed by the states at the start of
% each trajectory segment.
% There are constraints placed on both the initial and final states
% The simulation is implemented using 4th-order runge-kutta*.

nState = 4;  %Number of states in the problem (x,y,dx,dy)
nSegment = P.nSegment;
tEnd = decVar(1);
z0 = reshape(decVar(2:end),nState,P.nSegment);

% Run a simulation from the start of each segment, in parallel
nSub = P.nSubStep;  %Number of sub-steps for the integration method
tSim = linspace(0,tEnd/nSegment, nSub+1);
z = rk4_cannon(tSim,z0,P.c); %Simulate the trajectory
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

if nargout==4  %Only used for post-processing  --  return trajectory

    tTraj = zeros(1,nSegment*nSub+1);
    zTraj = zeros(nState,nSegment*nSub+1);
    
    %Stitch together the trajectory
    idx = 0;
    
    tSegment = linspace(0,tEnd,nSegment+1); tSegment(end) = [];
    for i=1:nSegment  %Slow looping, but only run once, so not too bad
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
%                     Data-logging and Diagnostics:                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function stop = outFun(decVar,optimVal,state)
% This function is used for logging the progress of fmincon throughout the
% optimization run. It does not affect the optimization process.

global ITER_LOG_MULTIPLESHOOTING;  %Keeping track of iteration details

stop = false;

switch state
    case 'init'
    case 'iter'
        iter = optimVal.iteration+1;
        ITER_LOG_MULTIPLESHOOTING(iter).optimVal = optimVal;
        ITER_LOG_MULTIPLESHOOTING(iter).decVar = decVar;
    case 'done'
    otherwise
end

end


