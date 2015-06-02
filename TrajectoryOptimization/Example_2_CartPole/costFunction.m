function cost = costFunction(x,u,p)
% cost = costFunction(x,u,p)
%
% For now, simple torque-squared cost function:
cost = u(1,:).^2 + u(2,:).^2;

end