function V = topple_contactVertical(th,P)

% AUTOMATICALLY GENERATED  --  DO NOT EDIT
% Computes vertical component of contact force for a stick toppling from rest.

m = P.m;
g = P.g;
L = P.L;
I = P.I;

V = (g.*m.*(I - 2.*L.^2.*m.*cos(th) + 3.*L.^2.*m.*cos(th).^2))./(I + L.^2.*m);

end
