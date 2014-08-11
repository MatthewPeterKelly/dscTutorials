function [dZ, C, E] = dynamics_slideNeg(~,Z,P)

% DO NOT EDIT
% This function was automatically generated

u = P.u; % Coefficient of friction
    dZ = zeros(size(Z));
    E = zeros(2,size(Z,2)); % [potential; kinetic]
    C = zeros(2,size(Z,2)); % [horizontal; vertical]
    
    m = P.m;
    g = P.g;
    L = P.L;
    I = P.I;
    
    th = Z(1,:);
    x = Z(2,:);
    dth = Z(3,:);
    dx = Z(4,:);
    
    dZ(1,:) = dth;
    dZ(2,:) = dx;

if isinf(u)
    dZ(3,:) = (2.*cos(th).*(g - L.*dth.^2.*cos(th)))./(L.*sin(2.*th));
    dZ(4,:) = -(L.^3.*dth.^2.*m.*cos(th) - L.^2.*g.*m - I.*g + L.^2.*g.*m.*sin(th).^2 + I.*L.*dth.^2.*cos(th))./(L.^2.*m.*cos(th).*sin(th));
    C(1,:) = (2.*I.*(g - L.*dth.^2.*cos(th)))./(L.^2.*sin(2.*th));
    C(2,:) = 0;

elseif u==0
    dZ(3,:) = (2.*L.*m.*sin(th).*(g - L.*dth.^2.*cos(th)))./(2.*I + L.^2.*m - L.^2.*m.*cos(2.*th));
    dZ(4,:) = -(L.^3.*dth.^2.*m.*sin(th) + I.*L.*dth.^2.*sin(th) - L.^2.*g.*m.*cos(th).*sin(th))./(I + L.^2.*m - L.^2.*m.*cos(th).^2);
    C(1,:) = 0;
    C(2,:) = (2.*I.*m.*(g - L.*dth.^2.*cos(th)))./(2.*I + L.^2.*m - L.^2.*m.*cos(2.*th));

else

    dZ(3,:) = (2.*L.*m.*(g - L.*dth.^2.*cos(th)).*(sin(th) + u.*cos(th)))./(2.*I + L.^2.*m - L.^2.*m.*cos(2.*th) + L.^2.*m.*u.*sin(2.*th));
    dZ(4,:) = -(L.^3.*dth.^2.*m.*sin(th) - I.*g.*u - L.^2.*g.*m.*u + I.*L.*dth.^2.*sin(th) + I.*L.*dth.^2.*u.*cos(th) + L.^3.*dth.^2.*m.*u.*cos(th) + L.^2.*g.*m.*u.*sin(th).^2 - L.^2.*g.*m.*cos(th).*sin(th))./(I + L.^2.*m - L.^2.*m.*cos(th).^2 + L.^2.*m.*u.*cos(th).*sin(th));
    C(1,:) = (2.*I.*m.*u.*(g - L.*dth.^2.*cos(th)))./(2.*I + L.^2.*m - L.^2.*m.*cos(2.*th) + L.^2.*m.*u.*sin(2.*th));
    C(2,:) = (2.*I.*m.*(g - L.*dth.^2.*cos(th)))./(2.*I + L.^2.*m - L.^2.*m.*cos(2.*th) + L.^2.*m.*u.*sin(2.*th));


end
    E(1,:) = L.*g.*m.*cos(th);
    E(2,:) = (m.*((dx - L.*dth.*cos(th)).^2 + L.^2.*dth.^2.*sin(th).^2))./2 + (I.*dth.^2)./2;
end
