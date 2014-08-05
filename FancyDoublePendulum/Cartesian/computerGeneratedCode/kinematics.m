function Kinematics = kinematics(States)
% function Kinematics = kinematics(States)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Power()
% 13-Dec-2013 11:41:00
%
%  ARGUMENTS: 
%   States = [Nstate x Ntime] matrix of states 
% 
%  RETURNS: 
%   Kinematics = a struct with fields:   
%      th1 = angle of leg 1   
%      th2 = angle of leg 2   
%      L1 = length of leg 1   
%      L2 = length of leg 2   
%      dth1 = (d/dt) angle of leg 1   
%      dth2 = (d/dt) angle of leg 2   
%      dL1 = (d/dt) length of leg 1   
%      dL2 = (d/dt) length of leg 2   
% 
%
% Matthew Kelly 
% Cornell University 
% 

% See also DERIVE_EOM 
% 
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

% Commonly used expressions
L1 = ((x0 - x1).^2 + (y0 - y1).^2).^(1./2);
L2 = ((x0 - x2).^2 + (y0 - y2).^2).^(1./2);
% dL1 = ((dx0 - dx1)*(x0 - x1) + (dy0 - dy1)*(y0 - y1))/L1;
% dL2 = ((dx0 - dx2)*(x0 - x2) + (dy0 - dy2)*(y0 - y2))/L2;
% th1 = atan2(x1 - x0, y1 - y0);
% th2 = atan2(x2 - x0, y2 - y0);
% dth1 = -((dy0 - dy1)*(x0 - x1) - (dx0 - dx1)*(y0 - y1))/L1^2;
% dth2 = -((dy0 - dy2)*(x0 - x2) - (dx0 - dx2)*(y0 - y2))/L2^2;

Kinematics.L1 = L1;
Kinematics.L2 = L2;
Kinematics.dL1 = ((dx0 - dx1).*(x0 - x1) + (dy0 - dy1).*(y0 - y1))./L1;
Kinematics.dL2 = ((dx0 - dx2).*(x0 - x2) + (dy0 - dy2).*(y0 - y2))./L2;
Kinematics.th1 = atan2(x1 - x0, y1 - y0);
Kinematics.th2 = atan2(x2 - x0, y2 - y0);
Kinematics.dth1 = -((dy0 - dy1).*(x0 - x1) - (dx0 - dx1).*(y0 - y1))./L1.^2;
Kinematics.dth2 = -((dy0 - dy2).*(x0 - x2) - (dx0 - dx2).*(y0 - y2))./L2.^2;

end
