function [time, state, control, exitFlag, output] = multipleShooting_Euler(bound, guess, param)
% problem = multipleShooting_Euler(bound, guess, param)
%
% This function implements all of the functions needed for multiple
% shooting using Euler's method for Transcription of the trajectory
% optimization problem. The results are returned as a "problem" struct,
% which can then be passed directly to FMINCON.
%
% NOTES:
%
%

nSegment = param.nSegment;
nSubStep = param.nSubStep;

%%% Decision Variables
%
%   T = [1,1] duration of the trajectory
%
%   X = [2, nSegment-1]
%   --> intermediate state variables. Boundary points are known, and thus
%   not included as decision variables.
%
%   U = [1, nSegment*nSubStep]
%   --> control for each intergration sub-step. We assume that the control
%   is a zero-order-hold, so we only need to control at the start of each
%   sub-step.
%
%%%

%%% Bounds on decision variables:
%
T_low = bound.timeLow;
X_low = bound.stateLow*ones(1,nSegment-1);
U_low = bound.controlLow*ones(1,nSegment*nSubStep);
zLow = packDecVar(T_low, X_low, U_low);

T_upp = bound.timeUpp;
X_upp = bound.stateUpp*ones(1,nSegment-1);
U_upp = bound.controlUpp*ones(1,nSegment*nSubStep);
zUpp = packDecVar(T_upp, X_upp, U_upp);
%
%%%

%%% Guess at the decision variables (initialization for FMINCON)
%
T_guess = guess.time(end);

tState = linspace(0,T_guess,nSegment+1); % Time at the boundaries of each segment
tState([1,end]) = [];  % Remove the initial and final times, since we already know the state there
X_guess = interp1(guess.time', guess.state',tState')';  %Interpolate guess at intermediate grid points

tControl = linspace(0,T_guess,nSegment*nSubStep+1); % Time at the boundaries of each segment
tControl(end) = [];  % no control at the final time
U_guess = interp1(guess.time', guess.control',tControl')';  %Interpolate guess at intermediate grid points

zGuess = packDecVar(T_guess, X_guess, U_guess);
%
%%%



%%% Create problem struct for fmincon
%
problem.options = optimset(...
    'Display','iter',... % {'final','iter'}
    'MaxFunEvals',1e4);

problem.x0 = zGuess;
problem.lb = zLow;
problem.ub = zUpp;
problem.Aineq = [];
problem.Aeq = [];
problem.bineq = [];
problem.beq = [];
problem.objective = @(z)( z(1) );  %Minimize trajectory duration
problem.nonlcon = @(z)( constraintFun(z,param) );
problem.solver = 'fmincon';
%
%%%

%%% Solve the non-linear program (NLP)
%
[zSoln, ~, exitFlag, output] = fmincon(problem);
%
%%%

[~,~,time,state,control] = problem.nonlcon(zSoln);  %Evaluate the solution


end



function z = packDecVar(T,X,U)
%
% This function takes the three types of decision variables and collapses
% them into a single column vector for FMINCON
%

z = [...
    T;
    reshape(X,numel(X),1);
    reshape(U,numel(U),1)];

end



function [T, X, U] = unpackDecVar(z, nSegment, nSubStep)
%
% This function takes the column vector of decision variables and unpacks
% it into the duration, state, and control matricies that are used to
% enforce the dynamics along the trajectory.
%

nx = 2*(nSegment-1);   %2 = dimension of the state space
nu = nSegment*nSubStep;

T = z(1);  %Duration

xIdx = 1 + (1:nx);
uIdx = 1 + nx + (1:nu);

X = reshape(z(xIdx),2,nSegment-1);
U = reshape(z(uIdx),1,nSegment*nSubStep);

end


function [c, ceq, t, x, u] = constraintFun(z,param)
%
% This function is called by FMINCON on each iteration as it solves the
% non-linear program (NLP). It enforces all non-linear constraints on the
% problem. In other words, it ensures that the dynamics are satisfied along
% the trajectory.
%
% The dynamics are enforced by a defect constraint. This constraint ensures
% that the end of each segment of the trajectory matches the beginning of
% the next. In the case of the last segment, it ensures that the boundary
% conditions are satisfied.
%

% Unpack the decision variables
nSegment = param.nSegment;
nSubStep = param.nSubStep;
[T, X, U] = unpackDecVar(z, nSegment, nSubStep);

h = T/(nSegment*nSubStep);  %integration time step

D = zeros(2,nSegment);  % Allocate memory for defects

x0 = [0;0];  %Initial state
xF = [0; param.riverWidth];  %Final state

Xlow = [x0, X];  % State at the start of each segment
Xupp = [X, xF];  % State at the end of each segment

x = zeros(2,nSegment*nSubStep + 1);  %Store state here

%%% Simulate the system forward using Euler's method.
%
% NOTE: This is NOT a fast or efficient method. I've written it out this
% way so that it is easier to see what is going on. For use with "real"
% problems, a higher order method, such as the Mid-Point Method or
% 4th-order Runge Kutta should be used. The resulting calculations should
% also be vectorized over each trajectory segment. Do not attempt to use a
% variable-order method (like ode45) instead of the fixed integration. The
% variable-order methods are accurate but NOT consistent, which causes
% convergence problems when it is passed to FMINCON. Check Bett's textbook:
% "Practical Methods for Optimal Control and Estimation Using Noninear
% Programing" for details.
%

idx = 0;
for i = 1:nSegment
    for j = 1:nSubStep
        idx = idx+1;
        
        % Reset the trajectory at the start of each segment
        if j==1  %Start of a segment
            xPrev = Xlow(:,i);
        else
            xPrev = xNext;
        end
        x(:,idx) = xPrev;
        
        %Propoagate dynamics
        dx = riverBoatDynamics(xPrev,U(idx),param);
        xNext = xPrev + h*dx;  %Euler Integration
        
        % compute defect if end of the segment
        if j==nSubStep  %Then last step!
            D(:,i) = xNext - Xupp(:,i);
        end
        
    end
end
x(:,end) = xF;

% Pack up for fmincon
c = [];  %No inequality constraints
ceq = reshape(D,numel(D),1);  %Reshape into a column vector

% Extra outputs, used to return the full solution.
t = linspace(0,T,nSegment*nSubStep+1);
u = [U, U(end)];  %Copy last entry for easy plotting

end


