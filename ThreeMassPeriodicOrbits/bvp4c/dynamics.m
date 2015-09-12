function dX = dynamics(X,P)
% dX = dynamics(X,P)
%
% This function computes the first-order dynamics for the classical
% "three-body" problem in newtonian mechanics: three point masses with
% inverse-square gravity.
%
% The input state contains only information about the first two particles.
% The state of the third particle is computed from the assumption (without
% loss of generality) that the center of mass of the system is at the
% origin and not moving.
%
% INPUTS:
%   X = [12, N] = first order state vector
%       X(1:3,:) = X1 = [x1;y1;z1]
%       X(4:6,:) = X2 = [x2;y2;z2]
%       X(7:9,:) = V1 = [dx1;dy1;dz1]
%       X(10:12,:) = V2 = [dx2;dy2;dz2]
%   P = parameter struct:
%       .G = gravity constant
%       .m1 = mass one
%       .m2 = mass two
%       .m3 = mass three
%
% OUTPUTS:
%   dX = [V1;V2; dV1; dV2];
%
%

% Positions
X1 = X(1:3,:);
X2 = X(4:6,:);

% Velocity
V1 = X(7:9,:);
V2 = X(10:12,:);

% Compute X3 from definition that CoM is at the origin
%   x = (x1*m1 + x2*m2 + x3*m3)/(m1 + m2 + m3);
X3 = -(X1*P.m1 + X2*P.m2)/P.m3;
% V3 = -(V1*P.m1 + V2*P.m2)/P.m3;   %Unused, but good to know

% Force Vectors
F12 = getForce(X1, X2, P.m1, P.m2, P.G);  % Acting on 1 from 2
F23 = getForce(X2, X3, P.m2, P.m3, P.G);  % Acting on 2 from 3
F13 = getForce(X1, X3, P.m1, P.m3, P.G);  % Acting on 1 from 3
    
% Acceleration Vectors
A1 = (F12 + F13)/P.m1;
A2 = (-F12 + F23)/P.m2;
% A3 = (-F23 - F13)/P.m3;   %Unused, but good to know

% Pack up solution:
dX = [V1; V2; A1; A2];

end


function F = getForce(Xa, Xb, ma, mb, G)
%
% Computes the force acting on particle a due to particle b, given
% parameters in the struct P
%

r = Xa - Xb;    %Vector from b to a
r2 = ones(3,1)*sum(r.^2,1);   % Length squared

lFl = G*ma*mb./r2;  %Force magnitude

F = -lFl .* (r./sqrt(r2));   %Force vector

end

