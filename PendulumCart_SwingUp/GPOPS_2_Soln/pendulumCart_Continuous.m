function output = pendulumCart_Continuous(input)

%Dynamical state
X = input.phase(1).state;  

%Control force
F = input.phase(1).control;

%Define Parameters
P = input.auxdata;    

%Call continuous dynamics function:
Xd = Pendulum_Cart_Dynamics_Forced(X',F',P)';

%Get the cost integrand:   (force-squared for now)
costIntegrand = F.^2;

%Pack up the output:
output.dynamics = Xd;
output.integrand = costIntegrand;

end