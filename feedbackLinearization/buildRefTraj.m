function ref = buildRefTraj(q,dq,ddq,ref)
% ref = buildRefTraj(q,dq,ddq,ref)
%
% This function computes the piecewise polynomial trajectories that make up
% the reference trajectory for the feedback linearization
%
% INPUTS:
%   q = [nConfig, nTime] = reference trajectory in configuration space
%   dq = [nConfig, nTime] = dq/dt = rates in time for ref traj
%   ddq = [nConfig, nTime] = ddq/ddt = accelerations for ref traj
%   ref = prototype for ref struct, with fields:
%       .wn = natural frequency of the controller
%       .xi = damping ratio of the controller
%       .c = [1, nConfig] = mapping from configuration to phase
%       .H = [nMeasure, nConfig] = mapping from configuration to measurement
%
% OUTPUTS:
%
%   ref = full ref struct, with added fields:
%       .pp = piecewise-polynomial trajectories for:
%           .h = reference measurement vector
%           .dh = dMeasurement/dPhase
%           .ddh = second derivative of measurements with respect to phase 
%           .dhdt = dMeasurement/dTime
%       
%

% Trajectories: measurement and phase vs time:
hRef = ref.H*q;   % Target measurement
dhRef = ref.H*dq;
ddhRef = ref.H*ddq;
pRef = ref.c*q; % Phase
dpRef = ref.c*dq;
ddpRef = ref.c*ddq;

% Compute derivatives wrt phase using chain rule:
dhRefdp = dhRef./dpRef;
ddhRefddp = (ddhRef - dhRefdp.*ddpRef)./(dpRef.^2);

% Represent trajectories as piecewise-polynomial
ref.pp.h = pchip(pRef,hRef);
ref.pp.dh = pchip(pRef,dhRefdp);   % Derivative wrt phase  (for ref traj)
ref.pp.dhdt = pchip(pRef,dhRef);   % Derivative wrt time   (for stabilization)
ref.pp.ddh = pchip(pRef,ddhRefddp);

end