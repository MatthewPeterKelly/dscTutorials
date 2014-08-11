function dth = topple_angularRate(th,P)

% AUTOMATICALLY GENERATED  --  DO NOT EDIT
% Computes angular rate for a stick toppling from rest.

m = P.m;
g = P.g;
L = P.L;
I = P.I;

dth = sign(th).*(2.^(1./2).*(1 - cos(th)).^(1./2).*(L.*g.*m.*(I + L.^2.*m)).^(1./2))./(I + L.^2.*m);

end
