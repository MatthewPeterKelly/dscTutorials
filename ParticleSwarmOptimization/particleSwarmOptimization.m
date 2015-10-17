function soln = particleSwarmOptimization(problem)
% soln = particleSwarmOptimization(problem)
%
% This function computes the solution to the optimization problem using
% particle swarm optimization
%
% INPUTS:
%   problem = struct that describes the optimization problem
%       .options.maxIter = number of iterations for the search
%       .options.populationCount = number of particles to use in search
%       .options.w = search parameter - damping
%       .options.pl = search paramter - local search
%       .options.pg = search parameter - global search
%       .xLow = [nDim, 1] = column vector for lower bound on initial search space 
%       .xUpp = [nDim, 1] = column vector for upper bound on initial search space 
%       .objFun = function handle:
%           f = objFun(x)
%               x = [nDim, nPop] = vectorized objective function
%               f = [1,nPop] = cost for each column in x
%          
% OUTPUTS:
%   soln = output struct with solution and history log
%       .x = best point found
%       .f = objective function at best point found
%       .log = history of the search, used for plotting and analysis
%       .problem = copy of the problem struct
%
% NOTES:
%   This is just a quick test function that I wrote to study how the
%   optimization parameters effect the behavior of the optimization, as
%   well as to figure out how it works with a noisy objective function. For
%   more rigerous optimization the code should be improved, most notably by
%   adding a convergence criteria and exit flag.
%
%

maxIter = problem.options.maxIter;
nPop = problem.options.populationCount;
nDim = length(problem.xLow);

% Memory allocation
Gx = zeros(nDim,1,maxIter);  %Global best solution point
Gf = zeros(1,1,maxIter);  %Global best solution value
Lx = zeros(nDim,nPop,maxIter);  %Local best solution
Lf = zeros(1,nPop,maxIter);  %Local best value
X = zeros(nDim,nPop,maxIter);  %Particle location
F = zeros(1,nPop,maxIter);  %objFun(X);
V = zeros(nDim,nPop,maxIter);  %Particle velocity

% Linear transform for vectorized random numbers
a = problem.xLow*ones(1,nPop);
b = (problem.xUpp-problem.xLow)*ones(1,nPop);

% Initialization:
X(:,:,1) = a + b.*rand(nDim,nPop);
V(:,:,1) = -abs(b) + 2*abs(b).*rand(nDim,nPop);
F(1,:,1) = problem.objFun(X(:,:,1));
Lx(:,:,1) = X(:,:,1);
Lf(1,:,1) = F(1,:,1);
[Gf(1,1,1),idx] = min(F(1,:,1));
Gx(:,1,1) = Lx(:,idx,1);

% Run the optimization here:
pl = problem.options.pl;
pg = problem.options.pg;
w = problem.options.w;
for i=2:maxIter
    rl = rand(nDim,nPop);  %Local random search dist
    rg = rand(nDim,nPop);  %global random search dist
    
    %%% Update all points
    V(:,:,i) = w*V(:,:,i-1) + ...    % Inertial terms
        pl.*rl.*(Lx(:,:,i-1)-X(:,:,i-1)) + ...  % Local search
        pg.*rg.*(Gx(:,1,i-1)*ones(1,nPop)-X(:,:,i-1));   %Global search
    X(:,:,i) = X(:,:,i-1) + V(:,:,i);  %Position update
    F(1,:,i) = problem.objFun(X(:,:,i));  %Objective function evaluation
    
    %%% Update local best:
    update = F(1,:,i) < Lf(1,:,i-1);  %Points that have new best values
    Lf(1,update,i) = F(1,update,i);
    Lf(1,~update,i) = Lf(1,~update,i-1);
    Lx(:,update,i) = X(:,update,i);
    Lx(:,~update,i) = Lx(:,~update,i);
    
    %%% Update global best:
    [Gf(1,1,i),idx] = min(Lf(1,:,i));
    Gx(:,1,i) = Lx(:,idx,i);
    
end

% Store the final solution
soln.x = reshape(Gx(:,1,end),nDim,1);
soln.f = Gf(end);

% Store all of the search history for analysis
soln.log.Gx = Gx;
soln.log.Gf = Gf;
soln.log.Lx = Lx;
soln.log.Lf = Lf;
soln.log.X = X;
soln.log.F = F;
soln.log.V = V;
soln.problem = problem;

end