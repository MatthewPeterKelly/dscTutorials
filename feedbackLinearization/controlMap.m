function u = controlMap(dq,v,dhRef,ddhRef,c,H,D,G,B)
%
%  This function maps a desired tracking error acceleration (v) to joint
%  torques, using feedback linearization / hybrid zero dynamics
%
% INPUTS:
%
%   dq = [nConfig, 1] = time derivative of configuration
%
%   v = [nMeasure, 1] = desired acceleration for error dynamics
%
%   dhRef = [nMeasure, 1] = change in reference trajectory wrt phase
%   ddhRef = [nMeasure, 1] = second derivative of ref. traj. wrt phase
%
%   c = [1, nConfig] = mapping from configuration to phase
%   H = [nMeasure, nConfig] = mapping from configuration to measurement
%
%   D = [nConfig, nConfig] = D(q) = mass-matrix for dynamics
%   G = [nConfig, 1] = G(q,dq) = state-based acceleration terms
%   B = [nConfig, nControl] = mapping from joint torques to accelerations
%
% OUTPUTS:
%
%   u = [nControl, 1] = torque vector
%
% NOTES:
%
%   Dynamics:   D*ddq + G = B*u
%
%   Tracking Error:   h = H*q - hRef
%
%   Phase:   c*q
%
%

% Temp variables to keep things readable:
T1 = H-dhRef*c;
T2 = v + ddhRef*(c*dq)^2;
T3 = T1\T2;
T4 = D*T3+G;

% Final calculation:
u = B\T4;

end