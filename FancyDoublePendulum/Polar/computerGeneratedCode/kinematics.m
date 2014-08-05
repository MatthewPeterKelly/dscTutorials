function [Position, Velocity, Energy] = kinematics(States, Parameters)
% function [Position, Velocity, Energy] = kinematics(States, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_Kinematics()
% 10-Dec-2013 19:40:47
%
%  RETURNS: 
%   Energy = a struct with fields: 
%       Potential 
%       Kinetic 
%       Total 
% 
%   Position = a struct with fields: 
%       footOne = [2xN] matrix with the position vectors for Foot One 
%       footTwo = [2xN] matrix with the position vectors for Foot Two 
%       hip = [2xN] matrix with the position vectors for the hip 
%       CoM = [2xN] matrix with the position vectors for the center of mass
% 
%   Velocity = a struct with fields: 
%       footOne = [2xN] matrix with the velocity vectors for Foot One 
%       footTwo = [2xN] matrix with the velocity vectors for Foot Two 
%       hip = [2xN] matrix with the velocity vectors for the hip 
%       CoM = [2xN] matrix with the velocity vectors for the center of mass 
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

m1 = Parameters.m1; % (kg) Foot One mass
m2 = Parameters.m2; % (kg) Foot Two mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

N = size(States,2);

Position.footOne = zeros(2,N);
Position.footTwo = zeros(2,N);
Position.hip = zeros(2,N);
Position.footOne(1,:) = x;
Position.footOne(2,:) = y;
Position.footTwo(1,:) = x + L1.*cos(th1) + L2.*cos(th2);
Position.footTwo(2,:) = y + L1.*sin(th1) + L2.*sin(th2);
Position.hip(1,:) = x + L1.*cos(th1);
Position.hip(2,:) = y + L1.*sin(th1);
Position.CoM = (m1*Position.footOne + m2*Position.footTwo + M*Position.hip)/(m1+m2+M);

if nargout > 1
    Velocity.footOne = zeros(2,N);
    Velocity.footTwo = zeros(2,N);
    Velocity.hip = zeros(2,N);
    Velocity.footOne(1,:) = dx;
    Velocity.footOne(2,:) = dy;
    Velocity.footTwo(1,:) = dx + dL1.*cos(th1) + dL2.*cos(th2) - L1.*dth1.*sin(th1) - L2.*dth2.*sin(th2);
    Velocity.footTwo(2,:) = dy + dL1.*sin(th1) + dL2.*sin(th2) + L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2);
    Velocity.hip(1,:) = dx + dL1.*cos(th1) - L1.*dth1.*sin(th1);
    Velocity.hip(2,:) = dy + dL1.*sin(th1) + L1.*dth1.*cos(th1);
    Velocity.CoM = (m1*Velocity.footOne + m2*Velocity.footTwo + M*Velocity.hip)/(m1+m2+M);

end
if nargout > 2
    Energy.Potential.m1 = g.*m1.*y;
    Energy.Potential.m2 = g.*m2.*(y + L1.*sin(th1) + L2.*sin(th2));
    Energy.Potential.M = M.*g.*(y + L1.*sin(th1));
    Energy.Potential.Total = Energy.Potential.m1 + Energy.Potential.m2 + Energy.Potential.M;

    Energy.Kinetic.m1 = (m1.*(dx.^2 + dy.^2))./2;
    Energy.Kinetic.m2 = (m2.*((dx + dL1.*cos(th1) + dL2.*cos(th2) - L1.*dth1.*sin(th1) - L2.*dth2.*sin(th2)).^2 + (dy + dL1.*sin(th1) + dL2.*sin(th2) + L1.*dth1.*cos(th1) + L2.*dth2.*cos(th2)).^2))./2;
    Energy.Kinetic.M = (M.*((dx + dL1.*cos(th1) - L1.*dth1.*sin(th1)).^2 + (dy + dL1.*sin(th1) + L1.*dth1.*cos(th1)).^2))./2;
    Energy.Kinetic.Total = Energy.Kinetic.m1 + Energy.Kinetic.m2 + Energy.Kinetic.M;

    Energy.Total = Energy.Kinetic.Total + Energy.Potential.Total;

end
end
