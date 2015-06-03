function cost = costFunction(t,x,u)
% cost = costFunction(x,u,config)
%
% For now, simple torque-squared cost function:
cost = 0.001*u(1,:).^2 + u(2,:).^2;

% Minimum acceleration of joint variables:
% ddz = chebyshevDerivative(x(3:4,:),[0,t]);
% cost = ddz(1,:).^2 + ddz(2,:).^2;
end