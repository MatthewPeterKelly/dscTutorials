function [dZ, C, E] = dynamics_hinge(~,Z,P)

% DO NOT EDIT
% This function was automatically generated

th = Z(1,:);
dth = Z(2,:);

m = P.m;
g = P.g;
L = P.L;
I = P.I;

dZ = zeros(size(Z));
dZ(1,:) = dth;
dZ(2,:) = (L.*g.*m.*sin(th))./(I + L.^2.*m);

% Contact Forces: 
C = zeros(2,length(th)); % [horizontal; vertical]
C(1,:) = (L.*m.*sin(th).*(I.*dth.^2 + L.^2.*dth.^2.*m - L.*g.*m.*cos(th)))./(I + L.^2.*m);
C(2,:) = (m.*(I.*g - L.^3.*dth.^2.*m.*cos(th) + L.^2.*g.*m.*cos(th).^2 - I.*L.*dth.^2.*cos(th)))./(I + L.^2.*m);

% Energy: 
E = zeros(2,length(th)); % [potential; kinetic]
E(1,:) = L.*g.*m.*cos(th);
E(2,:) = (m.*(L.^2.*dth.^2.*cos(th).^2 + L.^2.*dth.^2.*sin(th).^2))./2 + (I.*dth.^2)./2;

end
