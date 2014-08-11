function [dZ, E] = dynamics_flight(~,Z,P)

% DO NOT EDIT
% This function was automatically generated

th = Z(1,:);
x = Z(2,:);
y = Z(3,:);
dth = Z(4,:);
dx = Z(5,:);
dy = Z(6,:);

m = P.m;
g = P.g;
L = P.L;
I = P.I;

dZ = zeros(size(Z));
dZ(1,:) = dth;
dZ(2,:) = dx;
dZ(3,:) = dy;
dZ(4,:) = 0;
dZ(5,:) = -L.*dth.^2.*sin(th);
dZ(6,:) = L.*dth.^2.*cos(th) - g;

% Energy: 
E = zeros(2,length(th)); % [potential; kinetic]
E(1,:) = g.*m.*(y + L.*cos(th));
E(2,:) = (m.*((dx - L.*dth.*cos(th)).^2 + (dy - L.*dth.*sin(th)).^2))./2 + (I.*dth.^2)./2;

end
