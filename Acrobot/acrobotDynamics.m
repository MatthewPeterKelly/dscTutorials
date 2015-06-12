function dz = acrobotDynamics(z,u,p)
% dz = acrobotDynamics(z,u,p)
%
% This function computes the dynamics of the acrobot: double pendulum, two
% point masses, torque motor between links, no friction.
%
% INPUTS:
%   z = [4,1] = state vector
%   u = [1,1] = input torque
%   p = parameter struct:
%       .m1 = elbow mass
%       .m2 = wrist mass
%       .g = gravitational acceleration
%       .l1 = length shoulder to elbow
%       .l2 = length elbow to wrist
%
% OUTPUTS:
%   dz = [4,1] = dz/dt = time derivative of the state
% 
% NOTES:
%   
%   states:
%       1 = q1 = first link angle
%       2 = q2 = second link angle
%       3 = dq1 = first link angular rate
%       4 = dq2 = second link angular rate
%
%   angles: measured from negative j axis with positive convention
%




q1 = z(1);
q2 = z(2);
dq1 = z(3);
dq2 = z(4);

% D*ddq + G = B*u
[D,G,B] = autoGen_acrobotDynamics(q1,q2,dq1,dq2,p.m1,p.m2,p.g,p.l1,p.l2);

ddq = D\(B*u - G);

dz = [dq1;dq2;ddq];

end