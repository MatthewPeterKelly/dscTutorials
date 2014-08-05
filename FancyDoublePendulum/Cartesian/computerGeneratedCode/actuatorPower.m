function Power = actuatorPower(States, Actuators, Phase)
% function Power = actuatorPower(States, Actuators, Phase)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Power()
% 13-Dec-2013 11:41:00
%
%  ARGUMENTS: 
%   States = [Nstate x Ntime] matrix of states 
%   Actuator = [Nactuator x Ntime] matrix of actuator values 
%   Phase = {'D','F','S1',S2'} or omit. 
%       Since the ankle torques are acting against the ground, they should 
%       be set to zero when a given leg is in flight. If Phase is included, 
%       then this function will force these inputs to be zero. If it is 
%       ommitted, then it is assumed that the user has done that error 
%       checking. 
% 
%  RETURNS: 
%   Power = a struct with fields for each actuator: 
%       leg one (linear force actuator) 
%       leg two (linear force actuator) 
%       ankle one (torque actuator) 
%       ankle two (torque actuator) 
%       hip (torque from leg one on leg two 
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

if nargin==2
    Phase = '';
end
 
F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
if strcmp(Phase,'F') || strcmp(Phase, 'S2') 
    T1 = zeros(size(Actuators,1),1); %Foot One not in contact with ground!
else
    T1 = Actuators(:,3); % (Nm) External torque applied to Leg One
end
if strcmp(Phase,'F') || strcmp(Phase, 'S1') 
    T2 = zeros(size(Actuators,1),1); %Foot Two not in contact with ground!
else
    T2 = Actuators(:,4); % (Nm) External torque applied to Leg Two
end
Thip = Actuators(:,5); % (Nm) Torque acting on Leg Two from Leg One

% Intermediate Calculations
L1 = ((x0 - x1).^2 + (y0 - y1).^2).^(1./2);
L2 = ((x0 - x2).^2 + (y0 - y2).^2).^(1./2);
dL1 = ((dx0 - dx1).*(x0 - x1) + (dy0 - dy1).*(y0 - y1))./L1;
dL2 = ((dx0 - dx2).*(x0 - x2) + (dy0 - dy2).*(y0 - y2))./L2;
% th1 = atan2(x1 - x0, y1 - y0);
% th2 = atan2(x2 - x0, y2 - y0);
dth1 = -((dy0 - dy1).*(x0 - x1) - (dx0 - dx1).*(y0 - y1))./L1.^2;
dth2 = -((dy0 - dy2).*(x0 - x2) - (dx0 - dx2).*(y0 - y2))./L2.^2;

Power.legOne = F1.*dL1;
Power.legTwo = F2.*dL2;
Power.ankleOne = T1.*dth1;
Power.ankleTwo = T2.*dth2;
Power.hip = -Thip.*(dth1 - dth2);

end
