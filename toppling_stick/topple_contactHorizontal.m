function H = topple_contactHorizontal(th,P)

% AUTOMATICALLY GENERATED  --  DO NOT EDIT
% Computes horizontal component of contact force for a stick toppling from rest.

m = P.m;
g = P.g;
L = P.L;
I = P.I;

H = -(L.^2.*g.*m.^2.*(3.*sin(2.*th) - 4.*sin(th)))./(2.*(I + L.^2.*m));

end
