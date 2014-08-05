function [dStates, contactForces] = dynamics_singleStanceOne(States, Actuators, Parameters)
% function [dStates, Actuators] = dynamics_singleStanceOne(States, Actuators, Parameters)
%
% Computer Generated File -- DO NOT EDIT 
%
% This function was created by the function Write_ContinuousDynamics()
% 13-Dec-2013 11:41:00
%
% Dymanics Model: retractable double pendulum biped
% Motion Phase: Single Stance One
%
% Matthew Kelly 
% Cornell University 
% 

x0 = States(:,1); % (m) Hip horizontal position
y0 = States(:,2); % (m) Hip vertical position
x1 = States(:,3); % (m) Foot One horizontal position
y1 = States(:,4); % (m) Foot One vertical position
x2 = States(:,5); % (m) Foot Two horizontal position
y2 = States(:,6); % (m) Foot Two vertical position
% dx0 = States(:,7); % (m/s) Hip horizontal velocity
% dy0 = States(:,8); % (m/s) Hip vertical velocity
% dx1 = States(:,9); % (m/s) Foot One horizontal velocity
% dy1 = States(:,10); % (m/s) Foot One vertical velocity
% dx2 = States(:,11); % (m/s) Foot Two horizontal velocity
% dy2 = States(:,12); % (m/s) Foot Two vertical velocity

F1 = Actuators(:,1); % (N) Compresive axial force in Leg One
F2 = Actuators(:,2); % (N) Compresive axial force in Leg Two
T1 = Actuators(:,3); % (Nm) External torque applied to Leg One
T2 = 0; %Foot Two not in contact with ground!
Thip = Actuators(:,5); % (Nm) Torque acting on Leg Two from Leg One

m1 = Parameters.m1; % (kg) Foot One mass
m2 = Parameters.m2; % (kg) Foot Two mass
M = Parameters.M; % (kg) Hip mass
g = Parameters.g; % (m/s^2) Gravity

% Constraints for this phase: 
ddx1 = 0; %(m) Foot One horizontal position
ddy1 = 0; %(m) Foot One vertical position
H2 = 0; %(N) Foot Two, horizontal contact force
V2 = 0; %(N) Foot Two, vertical contact force

% Commonly used expressions
L1 = ((x0 - x1).^2 + (y0 - y1).^2).^(1./2);
L2 = ((x0 - x2).^2 + (y0 - y2).^2).^(1./2);
% dL1 = ((dx0 - dx1)*(x0 - x1) + (dy0 - dy1)*(y0 - y1))/L1;
% dL2 = ((dx0 - dx2)*(x0 - x2) + (dy0 - dy2)*(y0 - y2))/L2;
% th1 = atan2(x1 - x0, y1 - y0);
% th2 = atan2(x2 - x0, y2 - y0);
% dth1 = -((dy0 - dy1)*(x0 - x1) - (dx0 - dx1)*(y0 - y1))/L1^2;
% dth2 = -((dy0 - dy2)*(x0 - x2) - (dx0 - dx2)*(y0 - y2))/L2^2;
L1s = L1.^2;
L2s = L2.^2;

dStates = zeros(size(States));
dStates(:,1:6) = States(:,(1+6):(6+6));
dStates(:,7) = -(L1s.*T2.*y0 + L2s.*T1.*y0 - L2s.*T1.*y1 - L1s.*T2.*y2 - L1s.*Thip.*y0 + L2s.*Thip.*y0 + L1s.*Thip.*y2 - L2s.*Thip.*y1 - F1.*L1.*L2s.*x0 + F1.*L1.*L2s.*x1 - F2.*L1s.*L2.*x0 + F2.*L1s.*L2.*x2)./(L1s.*L2s.*M);
dStates(:,8) = -(L2s.*T1.*x1 - L2s.*T1.*x0 - L1s.*T2.*x0 + L1s.*T2.*x2 + L1s.*Thip.*x0 - L2s.*Thip.*x0 - L1s.*Thip.*x2 + L2s.*Thip.*x1 - F1.*L1.*L2s.*y0 + F1.*L1.*L2s.*y1 - F2.*L1s.*L2.*y0 + F2.*L1s.*L2.*y2 + L1s.*L2s.*M.*g)./(L1s.*L2s.*M);
dStates(:,9) = zeros(size(States,1),1);
dStates(:,10) = zeros(size(States,1),1);
dStates(:,11) = (T2.*y0 - T2.*y2 - Thip.*y0 + Thip.*y2 + H2.*L2s - F2.*L2.*x0 + F2.*L2.*x2)./(L2s.*m2);
dStates(:,12) = -(T2.*x0 - T2.*x2 - Thip.*x0 + Thip.*x2 - L2s.*V2 + F2.*L2.*y0 - F2.*L2.*y2 + L2s.*g.*m2)./(L2s.*m2);

% contactForces(:,1) == H1 == (N) Foot One, horizontal contact force
% contactForces(:,2) == V1 == (N) Foot One, vertical contact force
% contactForces(:,3) == H2 == (N) Foot Two, horizontal contact force
% contactForces(:,4) == V2 == (N) Foot Two, vertical contact force
contactForces = zeros(size(States,2),4);
contactForces(:,1) = (T1.*y1 - T1.*y0 - Thip.*y0 + Thip.*y1 + F1.*L1.*x0 - F1.*L1.*x1 + L1s.*ddx1.*m1)./L1s;
contactForces(:,2) = (T1.*x0 - T1.*x1 + Thip.*x0 - Thip.*x1 + F1.*L1.*y0 - F1.*L1.*y1 + L1s.*ddy1.*m1 + L1s.*g.*m1)./L1s;
contactForces(:,3) = zeros(size(States,1),1);
contactForces(:,4) = zeros(size(States,1),1);

end
