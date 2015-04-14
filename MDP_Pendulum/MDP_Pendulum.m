function [V, P, S, A] = MDP_Pendulum(pendulum)
% [V, P, S, A] = MDP_Pendulum(pendulum)
%
% This function solves a MDP using value iteration
%
% INPUTS:
%   pendulum = struct with information about pendulum model. 
%
% OUTPUTS:
%   V = [nBinState x 1] = value function for each state (by index)
%   P = [nBinState x 1] = policy for each state (by index)
%   S = [nState x nBinState] = discretized state space
%   A = [nAction x nBinAction] = discretized action space
%
% NOTES:
%   Suppose that iState is the index corresponding to some state
%   S(:,iState), then the value for this state would be V(iState) and the
%   optimal action would be A(:,P(iState));
%

% Build the MDP model of the pendulum (discretize problem)
[S,A,Ti,Tw,R] = MDP_Build_Pendulum(pendulum);

% Allocate memory for value iteration:
[~, nBinState] = size(S);
[~, nBinAction] = size(A);
V = zeros(nBinState,1);  %Value function
P = zeros(nBinState,1);  %Policy
VV = zeros(nBinState,nBinAction);  %Store temporary soln on each iteration

% Set up the optimization: 
gamma = pendulum.opt.discount^(pendulum.dyn.timeStep/pendulum.dyn.timeConstant);
tol = pendulum.opt.convergeTol;
maxIter = pendulum.opt.maxIter;
flagConverge = false;

% VALUE ITERATION 
for bigLoop = 1:maxIter   
    for aI = 1:nBinAction   % loop over actions, searching for the best
        VV(:,aI) = R(:,aI)...  %THIS IS THE KEY LINE
            + gamma*barycentricInterpolate_mex(V,Ti(:,:,aI),Tw(:,:,aI));
    end % aI  --  loop over actions
    [Vnew,P] = min(VV,[],2);  %Select best action for each state
    
    %Convergence stuff:  
    update = abs(Vnew-V);  %How much has the value function changed?
    V = Vnew;  % Copy the solution
    maxUpdate = max(update);
    if maxUpdate < tol
        disp('Successful Convergence!');
        flagConverge = true;
        break;  %Then converged within desired tolerance
    end
    if mod(bigLoop,pendulum.opt.dispMod)==0
        fprintf('Iter: %d, max update: %6.6f\n',bigLoop,maxUpdate);
    end
end % bigLoop -- main loop for value iteration

if ~flagConverge, disp('Reached Maximum Iteration Count'); end

end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function [S,A,Ti,Tw,R] = MDP_Build_Pendulum(pendulum)
%
% This function builds a Markov Decision Process (MDP) from a continuous
% model of a pendulum
%
%   S = [nState x nBinState] = discretized state space
%   A = [nAction x nBinAction] = discretized action space
%   Ti = [nBarycentric x nBinState x nBinAction] = transition index
%   Tw = [nBarycentric x nBinState x nBinAction] = transition weight
%   R = [nBinState x nBinAction] = reward matrix
%

% Information about the discretized grid:
thBnd = pendulum.grid.th.bnd;
thCount = pendulum.grid.th.nGrid;
wBnd = pendulum.grid.w.bnd;
wCount = pendulum.grid.w.nGrid;
uBnd = pendulum.grid.u.bnd;
uCount = pendulum.grid.u.nGrid;

% Build finite set of states and actions 
[th, w] = ndgrid(...
    linspace(thBnd(1),thBnd(2),thCount),...
    linspace(wBnd(1),wBnd(2),wCount));
S = [reshape(th,1,numel(th)); reshape(w,1,numel(w))];
sBnd = [thBnd;wBnd];
sN = [thCount;wCount];
A = linspace(uBnd(1),uBnd(2),uCount);

%%%%% Build the transition matrix:
nBinState = length(S);
nBinAction = length(A);
Tw = zeros(2+1,nBinState,nBinAction);  %Weights
Ti = zeros(2+1,nBinState,nBinAction);  %Index
dt = pendulum.dyn.timeStep;  %Time step for discretization
for aI = 1:nBinAction
    tau = A(aI);  %Torque   (Zero-order-hold)
    % 4th-order Runge-Kutta on dynamics 
    k1 = dynFun(S, tau, pendulum.dyn);
    k2 = dynFun(S + 0.5*dt*k1, tau, pendulum.dyn);
    k3 = dynFun(S + 0.5*dt*k2, tau, pendulum.dyn);
    k4 = dynFun(S + dt*k3, tau, pendulum.dyn);
    Snext = S + (dt/6)*(k1+2*k2+2*k3+k4);
    %Barycentric Interpolation: 
    [Tw(:,:,aI),Ti(:,:,aI)] = barycentricWeights_mex(Snext,sBnd,sN);
end % aI  --  loop over actions

%%%%% Build the reward (penalty) matrix:
th = S(1,:);
w = S(2,:);
R = zeros(nBinState,nBinAction);

% Penalty for being on the edge of the grid
thDel = pendulum.opt.cost.boundry.width*diff(thBnd);
wDel = pendulum.opt.cost.boundry.width*diff(wBnd);
pMax = pendulum.opt.cost.boundry.cost;
edgePenalty = smoothEdgePenalty(th,thBnd,thDel,pMax) + ...
    smoothEdgePenalty(w,wBnd,wDel,pMax);
R = R + edgePenalty'*ones(1,nBinAction);

% LQR cost function:
dt = pendulum.dyn.timeStep;
R = R + dt*(...
    (pendulum.opt.cost.angle*th.^2 + ...
    pendulum.opt.cost.rate*w.^2)'*ones(1,nBinAction));
R = R + dt*(...
    pendulum.opt.cost.torque*ones(nBinState,1)*A.^2);

end

function dX = dynFun(X,u,dyn)

dX = [X(2,:); pendulumDynamics(X(1,:),X(2,:),u,dyn)];

end






