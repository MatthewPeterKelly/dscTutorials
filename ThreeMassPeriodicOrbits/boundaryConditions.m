function C = boundaryConditions(X0, XF)
% C = boundaryConditions(X0, XF)
%
% This function computes the boundary conditions for periodic solutions.
%
% INPUTS:
%   X0 = Initial State
%   XF = Final State
%
% OUTPUTS:
%   C = defect constraint
%

C = X0 - XF;

end
