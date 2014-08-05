function Power = actuatorPower(States, Actuators, Phase)
% function Power = actuatorPower(States, Actuators, Phase)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Power()
% 10-Dec-2013 19:40:47
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
x = States(1,:); % (m) Foot One horizontal position
y = States(2,:); % (m) Foot One vertical position
th1 = States(3,:); % (rad) Leg One absolute angle
th2 = States(4,:); % (rad) Leg Two absolute angle
L1 = States(5,:); % (m) Leg One length
L2 = States(6,:); % (m) Leg Two length
dx = States(7,:); % (m/s) Foot One horizontal velocity
dy = States(8,:); % (m/s) Foot One vertical velocity
dth1 = States(9,:); % (rad/s) Leg One absolute angular rate
dth2 = States(10,:); % (rad/s) Leg Two absolute angular rate
dL1 = States(11,:); % (m/s) Leg One extension rate
dL2 = States(12,:); % (m/s) Leg Two extensioin rate

if nargin==2
    Phase = '';
end
 
F1 = Actuators(1,:); % (N) Compresive axial force in Leg One
F2 = Actuators(2,:); % (N) Compresive axial force in Leg Two
if strcmp(Phase,'F') || strcmp(Phase, 'S2') 
    T1 = zeros(1,size(Actuators,2)); %Foot One not in contact with ground!
else
    T1 = Actuators(3,:); % (Nm) External torque applied to Leg One
end
if strcmp(Phase,'F') || strcmp(Phase, 'S1') 
    T2 = zeros(1,size(Actuators,2)); %Foot Two not in contact with ground!
else
    T2 = Actuators(4,:); % (Nm) External torque applied to Leg Two
end
Thip = Actuators(5,:); % (Nm) Torque acting on Leg Two from Leg One

Power.legOne = F1.*dL1;
Power.legTwo = F2.*dL2;
Power.ankleOne = T1.*dth1;
Power.ankleTwo = T2.*dth2;
Power.hip = -Thip.*(dth1 - dth2);

end
