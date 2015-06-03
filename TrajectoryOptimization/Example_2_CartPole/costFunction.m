function cost = costFunction(t,x,u)
% cost = costFunction(x,u,config)
%

% du = chebyshevDerivative(u,[0,t]);
% [dz, ddz] = chebyshevDerivative(x(3:4,:),[0,t]);

cost = u.^2;
% cost = du.^2;
% cost = du.^2 + u.^2;
% cost = dz(1,:).^2 + dz(2,:).^2 + ddz(1,:).^2 + ddz(2,:).^2 +u.^2 + du.^2;


end