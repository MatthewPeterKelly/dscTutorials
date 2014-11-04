function u = controller(z,P)

%A simple controller for the force actuator that connects the point mass to
%the origin. It is just a PD-controller on the distance between the
%point-mass and the origin, with a non-linear term to compensate for
%dynamics of the system.

m = P.m;  %mass
g = P.g;  %Gravity
l = P.l;   %Nominal length
wn = P.freq;  %Response frequency
xi = P.damp;  %Damping ratio

r = z(1,:); %distance from point-mass to origin
th = z(2,:); %angle of vector between point-mass and origin
dr = z(3,:); %dr/dt
dth = z(4,:); %dth/dt

% Cancels out the nonlinear terms, drives ddr -> 0
% u - m*g*cos(th) = m*(ddr - r*dth*dth)
% u = m*g*cos(th) + m*(0 - r*dth*dth)
uNom = m*g*cos(th) - m*r.*dth.*dth;

%Linear control over transients
kp = wn^2;
kd = 2*xi*wn;
u = uNom + m*kp*(l - r) + m*kd*(0 - dr);

end
