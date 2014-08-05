function Energy = energy(States, Parameters)
% function Energy = energy(States, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Energy()
% 13-Dec-2013 11:41:00
%
%  RETURNS: 
%   Energy = a struct with fields: 
%       Potential 
%       Kinetic 
%       Total 
% 
%
% Matthew Kelly 
% Cornell University 
% 

% 
% See also DERIVE_EOM 
x0 = States(:,1); % (m) Hip horizontal position
y0 = States(:,2); % (m) Hip vertical position
x1 = States(:,3); % (m) Foot One horizontal position
y1 = States(:,4); % (m) Foot One vertical position
x2 = States(:,5); % (m) Foot Two horizontal position
y2 = States(:,6); % (m) Foot Two vertical position
dx0 = States(:,7); % (m/s) Hip horizontal velocity
dy0 = States(:,8); % (m/s) Hip vertical velocity
dx1 = States(:,9); % (m/s) Foot One horizontal velocity
dy1 = States(:,10); % (m/s) Foot One vertical velocity
dx2 = States(:,11); % (m/s) Foot Two horizontal velocity
dy2 = States(:,12); % (m/s) Foot Two vertical velocity

m1 = Parameters.m1; % (kg) Foot One mass
m2 = Parameters.m2; % (kg) Foot Two mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

Energy.Potential.m1 = g.*m1.*y1;
Energy.Potential.m2 = g.*m2.*y2;
Energy.Potential.M = M.*g.*y0;
Energy.Potential.Total = Energy.Potential.m1 + Energy.Potential.m2 + Energy.Potential.M;

Energy.Kinetic.m1 = (m1.*(dx1.^2 + dy1.^2))./2;
Energy.Kinetic.m2 = (m2.*(dx2.^2 + dy2.^2))./2;
Energy.Kinetic.M = (M.*(dx0.^2 + dy0.^2))./2;
Energy.Kinetic.Total = Energy.Kinetic.m1 + Energy.Kinetic.m2 + Energy.Kinetic.M;

Energy.Total = Energy.Kinetic.Total + Energy.Potential.Total;

end
