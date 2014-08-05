function [dStates, contactForces] = dynamics_singleStanceTwo(States, Actuators, Parameters)
% function [dStates, Actuators] = dynamics_singleStanceTwo(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_ContinuousDynamics()
% 10-Dec-2013 19:40:47
%
% Dymanics Model: retractable double pendulum biped
% Motion Phase: Single Stance Two
%
% Matthew Kelly 
% Cornell University 
% 

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

F1 = Actuators(1,:); % (N) Compresive axial force in Leg One
F2 = Actuators(2,:); % (N) Compresive axial force in Leg Two
T1 = 0; %Foot One not in contact with ground!
T2 = Actuators(4,:); % (Nm) External torque applied to Leg Two
Thip = Actuators(5,:); % (Nm) Torque acting on Leg Two from Leg One

m1 = Parameters.m1; % (kg) Foot One mass
m2 = Parameters.m2; % (kg) Foot Two mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

% Constraints for this phase: 
H1 = 0; %(N) Foot One, horizontal contact force
V1 = 0; %(N) Foot One, vertical contact force

dStates = zeros(size(States));
dStates(1:6,:) = States((1+6):(6+6),:);
dStates(7,:) = (T1.*sin(th1) - Thip.*sin(th1) + H1.*L1 - F1.*L1.*cos(th1))./(L1.*m1);
dStates(8,:) = -(T1.*cos(th1) - L1.*V1 - Thip.*cos(th1) + L1.*g.*m1 + F1.*L1.*sin(th1))./(L1.*m1);
dStates(9,:) = -(L2.*Thip.*m1 - L2.*T1.*m1 + L1.*T2.*m1.*cos(th1 - th2) + L1.*Thip.*m1.*cos(th1 - th2) - L2.*M.*T1.*cos(th1).^2 + L2.*M.*Thip.*cos(th1).^2 - L2.*M.*T1.*sin(th1).^2 + L2.*M.*Thip.*sin(th1).^2 + L1.*L2.*M.*V1.*cos(th1) - H1.*L1.*L2.*M.*sin(th1) - F2.*L1.*L2.*m1.*sin(th1 - th2) + 2.*L1.*L2.*M.*dL1.*dth1.*m1)./(L1.^2.*L2.*M.*m1);
dStates(10,:) = (L2.*M.*T1.*sin(th1).*sin(th2) - L2.*T1.*m1.*cos(th1).*cos(th2) - L2.*M.*Thip.*sin(th1).*sin(th2) + L2.*Thip.*m1.*cos(th1).*cos(th2) - L2.*T1.*m1.*sin(th1).*sin(th2) + L2.*Thip.*m1.*sin(th1).*sin(th2) - L2.*M.*T1.*cos(th1).^3.*cos(th2) + L2.*M.*Thip.*cos(th1).^3.*cos(th2) - L1.*L2.*M.*V1.*cos(th2) + H1.*L1.*L2.*M.*sin(th2) - L2.*M.*T1.*sin(th1).^3.*sin(th2) + L2.*M.*Thip.*sin(th1).^3.*sin(th2) + L2.*M.*T1.*cos(th1).*cos(th2) - L2.*M.*Thip.*cos(th1).*cos(th2) + L1.*T2.*m1.*cos(th1 - th2).*cos(th1).*cos(th2) + L1.*Thip.*m1.*cos(th1 - th2).*cos(th1).*cos(th2) - L2.*M.*T1.*cos(th1).*cos(th2).*sin(th1).^2 + L2.*M.*Thip.*cos(th1).*cos(th2).*sin(th1).^2 + L1.*T2.*m1.*cos(th1 - th2).*sin(th1).*sin(th2) - L1.*T2.*m1.*sin(th1 - th2).*cos(th1).*sin(th2) + L1.*T2.*m1.*sin(th1 - th2).*cos(th2).*sin(th1) + L1.*Thip.*m1.*cos(th1 - th2).*sin(th1).*sin(th2) - L1.*Thip.*m1.*sin(th1 - th2).*cos(th1).*sin(th2) + L1.*Thip.*m1.*sin(th1 - th2).*cos(th2).*sin(th1) - F1.*L1.*L2.*M.*cos(th1).*sin(th2) + F1.*L1.*L2.*M.*cos(th2).*sin(th1) - L2.*M.*T1.*cos(th1).^2.*sin(th1).*sin(th2) + L2.*M.*Thip.*cos(th1).^2.*sin(th1).*sin(th2) + F1.*L1.*L2.*m1.*cos(th1).*sin(th2) - F1.*L1.*L2.*m1.*cos(th2).*sin(th1) - F1.*L1.*L2.*M.*cos(th2).*sin(th1).^3 + F1.*L1.*L2.*M.*cos(th1).^3.*sin(th2) + L1.*L2.*M.*V1.*cos(th1).^2.*cos(th2) - H1.*L1.*L2.*M.*cos(th1).^2.*sin(th2) + L1.*L2.*M.*V1.*cos(th2).*sin(th1).^2 - H1.*L1.*L2.*M.*sin(th1).^2.*sin(th2) + L1.*L2.*M.*g.*m1.*cos(th2) - F2.*L1.*L2.*m1.*cos(th1 - th2).*cos(th1).*sin(th2) + F2.*L1.*L2.*m1.*cos(th1 - th2).*cos(th2).*sin(th1) - F2.*L1.*L2.*m1.*sin(th1 - th2).*cos(th1).*cos(th2) - F1.*L1.*L2.*M.*cos(th1).^2.*cos(th2).*sin(th1) + F1.*L1.*L2.*M.*cos(th1).*sin(th1).^2.*sin(th2) - F2.*L1.*L2.*m1.*sin(th1 - th2).*sin(th1).*sin(th2) - 2.*L1.*L2.*M.*dL2.*dth2.*m1.*cos(th2).^2 - 2.*L1.*L2.*M.*dL2.*dth2.*m1.*sin(th2).^2)./(L1.*L2.^2.*M.*m1.*(cos(th2).^2 + sin(th2).^2));
dStates(11,:) = -(T2.*m1.*sin(th1 - th2) + Thip.*m1.*sin(th1 - th2) - F1.*L2.*m1 + H1.*L2.*M.*cos(th1) + L2.*M.*V1.*sin(th1) + F2.*L2.*m1.*cos(th1 - th2) - F1.*L2.*M.*cos(th1).^2 - F1.*L2.*M.*sin(th1).^2 - L1.*L2.*M.*dth1.^2.*m1)./(L2.*M.*m1);
dStates(12,:) = (L2.*M.*T1.*cos(th1).*sin(th2) - L2.*M.*T1.*cos(th2).*sin(th1) - L2.*M.*Thip.*cos(th1).*sin(th2) + L2.*M.*Thip.*cos(th2).*sin(th1) - L2.*T1.*m1.*cos(th1).*sin(th2) + L2.*T1.*m1.*cos(th2).*sin(th1) + L2.*Thip.*m1.*cos(th1).*sin(th2) - L2.*Thip.*m1.*cos(th2).*sin(th1) - H1.*L1.*L2.*M.*cos(th2) + L2.*M.*T1.*cos(th2).*sin(th1).^3 - L2.*M.*T1.*cos(th1).^3.*sin(th2) - L2.*M.*Thip.*cos(th2).*sin(th1).^3 + L2.*M.*Thip.*cos(th1).^3.*sin(th2) - L1.*L2.*M.*V1.*sin(th2) + L1.*L2.*M.*g.*m1.*sin(th2) + L1.*L2.^2.*M.*dth2.^2.*m1.*cos(th2).^2 + L1.*L2.^2.*M.*dth2.^2.*m1.*sin(th2).^2 + L1.*T2.*m1.*cos(th1 - th2).*cos(th1).*sin(th2) - L1.*T2.*m1.*cos(th1 - th2).*cos(th2).*sin(th1) + L1.*T2.*m1.*sin(th1 - th2).*cos(th1).*cos(th2) + L1.*Thip.*m1.*cos(th1 - th2).*cos(th1).*sin(th2) - L1.*Thip.*m1.*cos(th1 - th2).*cos(th2).*sin(th1) + L1.*Thip.*m1.*sin(th1 - th2).*cos(th1).*cos(th2) + F1.*L1.*L2.*M.*cos(th1).*cos(th2) + L2.*M.*T1.*cos(th1).^2.*cos(th2).*sin(th1) - L2.*M.*Thip.*cos(th1).^2.*cos(th2).*sin(th1) - L2.*M.*T1.*cos(th1).*sin(th1).^2.*sin(th2) + L2.*M.*Thip.*cos(th1).*sin(th1).^2.*sin(th2) + L1.*T2.*m1.*sin(th1 - th2).*sin(th1).*sin(th2) + L1.*Thip.*m1.*sin(th1 - th2).*sin(th1).*sin(th2) + F1.*L1.*L2.*M.*sin(th1).*sin(th2) - F1.*L1.*L2.*m1.*cos(th1).*cos(th2) - F1.*L1.*L2.*m1.*sin(th1).*sin(th2) - F1.*L1.*L2.*M.*cos(th1).^3.*cos(th2) + H1.*L1.*L2.*M.*cos(th1).^2.*cos(th2) + H1.*L1.*L2.*M.*cos(th2).*sin(th1).^2 - F1.*L1.*L2.*M.*sin(th1).^3.*sin(th2) + L1.*L2.*M.*V1.*cos(th1).^2.*sin(th2) + L1.*L2.*M.*V1.*sin(th1).^2.*sin(th2) + F2.*L1.*L2.*m1.*cos(th1 - th2).*cos(th1).*cos(th2) - F1.*L1.*L2.*M.*cos(th1).*cos(th2).*sin(th1).^2 + F2.*L1.*L2.*m1.*cos(th1 - th2).*sin(th1).*sin(th2) - F2.*L1.*L2.*m1.*sin(th1 - th2).*cos(th1).*sin(th2) + F2.*L1.*L2.*m1.*sin(th1 - th2).*cos(th2).*sin(th1) - F1.*L1.*L2.*M.*cos(th1).^2.*sin(th1).*sin(th2))./(L1.*L2.*M.*m1.*(cos(th2).^2 + sin(th2).^2));

% contactForces(1,:) == H1 == (N) Foot One, horizontal contact force
% contactForces(2,:) == V1 == (N) Foot One, vertical contact force
% contactForces(3,:) == H2 == (N) Foot Two, horizontal contact force
% contactForces(4,:) == V2 == (N) Foot Two, vertical contact force
contactForces = zeros(4,size(States,2));
contactForces(1,:) = zeros(1,size(States,2));
contactForces(2,:) = zeros(1,size(States,2));
contactForces(3,:) = (L2.*M.*T1.*m2.*sin(th1).^3 - L2.*M.*Thip.*m2.*sin(th1).^3 + L1.*M.*T2.*m1.*sin(th2) - L2.*M.*T1.*m2.*sin(th1) + L1.*M.*Thip.*m1.*sin(th2) + L2.*M.*Thip.*m2.*sin(th1) + L2.*T1.*m1.*m2.*sin(th1) - L2.*Thip.*m1.*m2.*sin(th1) - H1.*L1.*L2.*M.*m2 + L1.*T2.*m1.*m2.*cos(th1 - th2).^2.*sin(th2) + L1.*Thip.*m1.*m2.*cos(th1 - th2).^2.*sin(th2) - F1.*L1.*L2.*M.*m2.*cos(th1).^3 + H1.*L1.*L2.*M.*m2.*cos(th1).^2 + H1.*L1.*L2.*M.*m2.*cos(th2).^2 + L1.*T2.*m1.*m2.*sin(th1 - th2).^2.*sin(th2) + L1.*Thip.*m1.*m2.*sin(th1 - th2).^2.*sin(th2) + H1.*L1.*L2.*M.*m2.*sin(th1).^2 + H1.*L1.*L2.*M.*m2.*sin(th2).^2 - L1.*T2.*m1.*m2.*cos(th1 - th2).*sin(th1) + L1.*T2.*m1.*m2.*sin(th1 - th2).*cos(th1) - L2.*T1.*m1.*m2.*cos(th1 - th2).*sin(th2) - L2.*T1.*m1.*m2.*sin(th1 - th2).*cos(th2) - L1.*Thip.*m1.*m2.*cos(th1 - th2).*sin(th1) + L1.*Thip.*m1.*m2.*sin(th1 - th2).*cos(th1) + L2.*Thip.*m1.*m2.*cos(th1 - th2).*sin(th2) + L2.*Thip.*m1.*m2.*sin(th1 - th2).*cos(th2) + F1.*L1.*L2.*M.*m2.*cos(th1) - F2.*L1.*L2.*M.*m1.*cos(th2) + L2.*M.*T1.*m2.*cos(th1).^2.*sin(th1) + L2.*M.*T1.*m2.*cos(th2).^2.*sin(th1) - L2.*M.*Thip.*m2.*cos(th1).^2.*sin(th1) - L2.*M.*Thip.*m2.*cos(th2).^2.*sin(th1) + L2.*M.*T1.*m2.*sin(th1).*sin(th2).^2 - L2.*M.*Thip.*m2.*sin(th1).*sin(th2).^2 - F1.*L1.*L2.*m1.*m2.*cos(th1) - F1.*L1.*L2.*M.*m2.*cos(th1).*cos(th2).^2 - F1.*L1.*L2.*M.*m2.*cos(th1).*sin(th1).^2 - F1.*L1.*L2.*M.*m2.*cos(th1).*sin(th2).^2 - F1.*L1.*L2.*m1.*m2.*sin(th1 - th2).*sin(th2) + F2.*L1.*L2.*m1.*m2.*sin(th1 - th2).*sin(th1) - F2.*L1.*L2.*m1.*m2.*cos(th1 - th2).^2.*cos(th2) - F2.*L1.*L2.*m1.*m2.*sin(th1 - th2).^2.*cos(th2) - L2.*M.*T1.*m2.*cos(th1 - th2).*cos(th1).^2.*sin(th2) - L2.*M.*T1.*m2.*sin(th1 - th2).*cos(th1).^2.*cos(th2) + L2.*M.*Thip.*m2.*cos(th1 - th2).*cos(th1).^2.*sin(th2) + L2.*M.*Thip.*m2.*sin(th1 - th2).*cos(th1).^2.*cos(th2) - L2.*M.*T1.*m2.*cos(th1 - th2).*sin(th1).^2.*sin(th2) - L2.*M.*T1.*m2.*sin(th1 - th2).*cos(th2).*sin(th1).^2 + L2.*M.*Thip.*m2.*cos(th1 - th2).*sin(th1).^2.*sin(th2) + L2.*M.*Thip.*m2.*sin(th1 - th2).*cos(th2).*sin(th1).^2 + F1.*L1.*L2.*m1.*m2.*cos(th1 - th2).*cos(th2) + F2.*L1.*L2.*m1.*m2.*cos(th1 - th2).*cos(th1) + F1.*L1.*L2.*M.*m2.*cos(th1 - th2).*cos(th1).^2.*cos(th2) + F1.*L1.*L2.*M.*m2.*cos(th1 - th2).*cos(th2).*sin(th1).^2 - F1.*L1.*L2.*M.*m2.*sin(th1 - th2).*cos(th1).^2.*sin(th2) - F1.*L1.*L2.*M.*m2.*sin(th1 - th2).*sin(th1).^2.*sin(th2) - H1.*L1.*L2.*M.*m2.*cos(th1 - th2).*cos(th1).*cos(th2) + L1.*L2.*M.*V1.*m2.*cos(th1 - th2).*cos(th1).*sin(th2) - L1.*L2.*M.*V1.*m2.*cos(th1 - th2).*cos(th2).*sin(th1) + L1.*L2.*M.*V1.*m2.*sin(th1 - th2).*cos(th1).*cos(th2) - H1.*L1.*L2.*M.*m2.*cos(th1 - th2).*sin(th1).*sin(th2) + H1.*L1.*L2.*M.*m2.*sin(th1 - th2).*cos(th1).*sin(th2) - H1.*L1.*L2.*M.*m2.*sin(th1 - th2).*cos(th2).*sin(th1) + L1.*L2.*M.*V1.*m2.*sin(th1 - th2).*sin(th1).*sin(th2))./(L1.*L2.*M.*m1.*(cos(th2).^2 + sin(th2).^2));
contactForces(4,:) = -(L1.*L2.*M.*V1.*m2 + L1.*M.*T2.*m1.*cos(th2) - L2.*M.*T1.*m2.*cos(th1) + L1.*M.*Thip.*m1.*cos(th2) + L2.*M.*Thip.*m2.*cos(th1) + L2.*T1.*m1.*m2.*cos(th1) - L2.*Thip.*m1.*m2.*cos(th1) + L2.*M.*T1.*m2.*cos(th1).^3 - L2.*M.*Thip.*m2.*cos(th1).^3 + L1.*T2.*m1.*m2.*cos(th1 - th2).^2.*cos(th2) + L1.*Thip.*m1.*m2.*cos(th1 - th2).^2.*cos(th2) + L1.*T2.*m1.*m2.*sin(th1 - th2).^2.*cos(th2) + L1.*Thip.*m1.*m2.*sin(th1 - th2).^2.*cos(th2) + F1.*L1.*L2.*M.*m2.*sin(th1).^3 - L1.*L2.*M.*V1.*m2.*cos(th1).^2 - L1.*L2.*M.*V1.*m2.*cos(th2).^2 - L1.*L2.*M.*V1.*m2.*sin(th1).^2 - L1.*L2.*M.*V1.*m2.*sin(th2).^2 - L1.*L2.*M.*g.*m1.*m2 - L1.*T2.*m1.*m2.*cos(th1 - th2).*cos(th1) - L2.*T1.*m1.*m2.*cos(th1 - th2).*cos(th2) - L1.*Thip.*m1.*m2.*cos(th1 - th2).*cos(th1) + L2.*Thip.*m1.*m2.*cos(th1 - th2).*cos(th2) + L2.*M.*T1.*m2.*cos(th1).*cos(th2).^2 - L2.*M.*Thip.*m2.*cos(th1).*cos(th2).^2 + L2.*M.*T1.*m2.*cos(th1).*sin(th1).^2 + L2.*M.*T1.*m2.*cos(th1).*sin(th2).^2 - L2.*M.*Thip.*m2.*cos(th1).*sin(th1).^2 - L2.*M.*Thip.*m2.*cos(th1).*sin(th2).^2 - L1.*T2.*m1.*m2.*sin(th1 - th2).*sin(th1) + L2.*T1.*m1.*m2.*sin(th1 - th2).*sin(th2) - L1.*Thip.*m1.*m2.*sin(th1 - th2).*sin(th1) - L2.*Thip.*m1.*m2.*sin(th1 - th2).*sin(th2) - F1.*L1.*L2.*M.*m2.*sin(th1) + F2.*L1.*L2.*M.*m1.*sin(th2) + F1.*L1.*L2.*m1.*m2.*sin(th1) + L2.*M.*T1.*m2.*sin(th1 - th2).*sin(th1).^2.*sin(th2) - L2.*M.*Thip.*m2.*sin(th1 - th2).*sin(th1).^2.*sin(th2) - F1.*L1.*L2.*m1.*m2.*cos(th1 - th2).*sin(th2) - F1.*L1.*L2.*m1.*m2.*sin(th1 - th2).*cos(th2) - F2.*L1.*L2.*m1.*m2.*cos(th1 - th2).*sin(th1) + F2.*L1.*L2.*m1.*m2.*sin(th1 - th2).*cos(th1) + F1.*L1.*L2.*M.*m2.*cos(th1).^2.*sin(th1) + F1.*L1.*L2.*M.*m2.*cos(th2).^2.*sin(th1) + F1.*L1.*L2.*M.*m2.*sin(th1).*sin(th2).^2 + F2.*L1.*L2.*m1.*m2.*cos(th1 - th2).^2.*sin(th2) + F2.*L1.*L2.*m1.*m2.*sin(th1 - th2).^2.*sin(th2) - L2.*M.*T1.*m2.*cos(th1 - th2).*cos(th1).^2.*cos(th2) + L2.*M.*Thip.*m2.*cos(th1 - th2).*cos(th1).^2.*cos(th2) - L2.*M.*T1.*m2.*cos(th1 - th2).*cos(th2).*sin(th1).^2 + L2.*M.*Thip.*m2.*cos(th1 - th2).*cos(th2).*sin(th1).^2 + L2.*M.*T1.*m2.*sin(th1 - th2).*cos(th1).^2.*sin(th2) - L2.*M.*Thip.*m2.*sin(th1 - th2).*cos(th1).^2.*sin(th2) - F1.*L1.*L2.*M.*m2.*cos(th1 - th2).*cos(th1).^2.*sin(th2) - F1.*L1.*L2.*M.*m2.*sin(th1 - th2).*cos(th1).^2.*cos(th2) - F1.*L1.*L2.*M.*m2.*cos(th1 - th2).*sin(th1).^2.*sin(th2) - F1.*L1.*L2.*M.*m2.*sin(th1 - th2).*cos(th2).*sin(th1).^2 + L1.*L2.*M.*V1.*m2.*cos(th1 - th2).*cos(th1).*cos(th2) + H1.*L1.*L2.*M.*m2.*cos(th1 - th2).*cos(th1).*sin(th2) - H1.*L1.*L2.*M.*m2.*cos(th1 - th2).*cos(th2).*sin(th1) + H1.*L1.*L2.*M.*m2.*sin(th1 - th2).*cos(th1).*cos(th2) + L1.*L2.*M.*V1.*m2.*cos(th1 - th2).*sin(th1).*sin(th2) - L1.*L2.*M.*V1.*m2.*sin(th1 - th2).*cos(th1).*sin(th2) + L1.*L2.*M.*V1.*m2.*sin(th1 - th2).*cos(th2).*sin(th1) + H1.*L1.*L2.*M.*m2.*sin(th1 - th2).*sin(th1).*sin(th2))./(L1.*L2.*M.*m1.*(cos(th2).^2 + sin(th2).^2));

end
