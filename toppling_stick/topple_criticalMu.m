function mu = topple_criticalMu(th,P)

% AUTOMATICALLY GENERATED  --  DO NOT EDIT
% Computes angular rate for a stick toppling from rest.

m = P.m;
g = P.g;
L = P.L;
I = P.I;

mu = -(L.^2.*m.*(4.*sin(th) - 6.*cos(th).*sin(th)))./(2.*(I - 2.*L.^2.*m.*cos(th) + 3.*L.^2.*m.*cos(th).^2));

end
