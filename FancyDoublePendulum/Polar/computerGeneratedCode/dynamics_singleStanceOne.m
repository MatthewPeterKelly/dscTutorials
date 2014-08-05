function [dStates, contactForces] = dynamics_singleStanceOne(States, Actuators, Parameters)
% function [dStates, Actuators] = dynamics_singleStanceOne(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_ContinuousDynamics()
% 10-Dec-2013 19:40:47
%
% Dymanics Model: retractable double pendulum biped
% Motion Phase: Single Stance One
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
T1 = Actuators(3,:); % (Nm) External torque applied to Leg One
T2 = 0; %Foot Two not in contact with ground!
Thip = Actuators(5,:); % (Nm) Torque acting on Leg Two from Leg One

m1 = Parameters.m1; % (kg) Foot One mass
m2 = Parameters.m2; % (kg) Foot Two mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

% Constraints for this phase: 
ddx = 0; %(m) Foot One horizontal position
ddy = 0; %(m) Foot One vertical position
H2 = 0; %(N) Foot Two, horizontal contact force
V2 = 0; %(N) Foot Two, vertical contact force

dStates = zeros(size(States));
dStates(1:6,:) = States((1+6):(6+6),:);
dStates(7,:) = zeros(1,size(States,2));
dStates(8,:) = zeros(1,size(States,2));
dStates(9,:) = -(L2.*Thip - L2.*T1 + L1.*T2.*cos(th1 - th2) + L1.*Thip.*cos(th1 - th2) - F2.*L1.*L2.*sin(th1 - th2) + 2.*L1.*L2.*M.*dL1.*dth1 + L1.*L2.*M.*ddy.*cos(th1) + L1.*L2.*M.*g.*cos(th1) - L1.*L2.*M.*ddx.*sin(th1))./(L1.^2.*L2.*M);
dStates(10,:) = (L1.*M.*T2 + L1.*M.*Thip - L2.*T1.*m2.*cos(th1 - th2) + L2.*Thip.*m2.*cos(th1 - th2) + L1.*T2.*m2.*cos(th1 - th2).^2 + L1.*Thip.*m2.*cos(th1 - th2).^2 + L1.*T2.*m2.*sin(th1 - th2).^2 + L1.*Thip.*m2.*sin(th1 - th2).^2 + L1.*L2.*M.*V2.*cos(th2) - H2.*L1.*L2.*M.*sin(th2) - F1.*L1.*L2.*m2.*sin(th1 - th2) - 2.*L1.*L2.*M.*dL2.*dth2.*m2 - L1.*L2.*M.*ddy.*m2.*cos(th2) - L1.*L2.*M.*g.*m2.*cos(th2) + L1.*L2.*M.*ddx.*m2.*sin(th2) - L1.*L2.*M.*ddx.*m2.*cos(th1 - th2).*sin(th1) + L1.*L2.*M.*ddx.*m2.*sin(th1 - th2).*cos(th1) + L1.*L2.*M.*ddy.*m2.*sin(th1 - th2).*sin(th1) + L1.*L2.*M.*g.*m2.*sin(th1 - th2).*sin(th1) + L1.*L2.*M.*ddy.*m2.*cos(th1 - th2).*cos(th1) + L1.*L2.*M.*g.*m2.*cos(th1 - th2).*cos(th1))./(L1.*L2.^2.*M.*m2);
dStates(11,:) = -(T2.*sin(th1 - th2) + Thip.*sin(th1 - th2) - F1.*L2 + F2.*L2.*cos(th1 - th2) - L1.*L2.*M.*dth1.^2 + L2.*M.*ddx.*cos(th1) + L2.*M.*ddy.*sin(th1) + L2.*M.*g.*sin(th1))./(L2.*M);
dStates(12,:) = (T1.*m2.*sin(th1 - th2) - Thip.*m2.*sin(th1 - th2) + F2.*L1.*M + H2.*L1.*M.*cos(th2) + L1.*M.*V2.*sin(th2) - F1.*L1.*m2.*cos(th1 - th2) + F2.*L1.*m2.*cos(th1 - th2).^2 + F2.*L1.*m2.*sin(th1 - th2).^2 + L1.*L2.*M.*dth2.^2.*m2 - L1.*M.*ddx.*m2.*cos(th2) - L1.*M.*ddy.*m2.*sin(th2) - L1.*M.*g.*m2.*sin(th2) + L1.*M.*ddx.*m2.*cos(th1 - th2).*cos(th1) + L1.*M.*ddy.*m2.*cos(th1 - th2).*sin(th1) - L1.*M.*ddy.*m2.*sin(th1 - th2).*cos(th1) + L1.*M.*g.*m2.*cos(th1 - th2).*sin(th1) - L1.*M.*g.*m2.*sin(th1 - th2).*cos(th1) + L1.*M.*ddx.*m2.*sin(th1 - th2).*sin(th1))./(L1.*M.*m2);

% contactForces(1,:) == H1 == (N) Foot One, horizontal contact force
% contactForces(2,:) == V1 == (N) Foot One, vertical contact force
% contactForces(3,:) == H2 == (N) Foot Two, horizontal contact force
% contactForces(4,:) == V2 == (N) Foot Two, vertical contact force
contactForces = zeros(4,size(States,2));
contactForces(1,:) = (Thip.*sin(th1) - T1.*sin(th1) + L1.*ddx.*m1 + F1.*L1.*cos(th1))./L1;
contactForces(2,:) = (T1.*cos(th1) - Thip.*cos(th1) + L1.*ddy.*m1 + L1.*g.*m1 + F1.*L1.*sin(th1))./L1;
contactForces(3,:) = zeros(1,size(States,2));
contactForces(4,:) = zeros(1,size(States,2));

end
