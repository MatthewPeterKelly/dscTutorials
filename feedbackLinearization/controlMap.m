function u = controlMap(dq, v, dhRef, ddhRef, c, H, D, B, G)
% u = controlMap(dq, v, dhRef, ddhRef, c, H, D, B, G)
%
% This function computes the control mapping from v (acceleration of error)
% to u (joint torques)
%
% INPUTS:
%
%   dq = [nConfig, 1] = time rate of change in configuration
%   v = [nMeasure, 1] = desired acceleration of error
%   
%   dhRef = [nMeasure, 1] = derivative of reference wrt phase
%   ddhRef = [nMeasure, 1] = second derivative of ref wrt phase
%
%   c = [1, nConfig] = mapping from configuration to phase
%   H = [nMeasure, nConfig] = mapping from configuration to measurement
%
%   {D, B, G} = Dynamics Matricies:    D*ddq + G = B*u
%       D = [nConfig, nConfig]  =  mass matrix
%       B = [nConfig, nControl] =  control matrix
%       G = [nConfig, 1] = everything else
%
%
% OUTPUTS:
%   
% u = [nControl, 1] = joint torque matrix
%

% Temp variables to keep the code readable:
T1 = H-dhRef*c;
T2 = T1/D;
T3 = v + ddhRef*(c*dq)^2;
T4 = T2\T3;

u = B\(T4 + G);

end