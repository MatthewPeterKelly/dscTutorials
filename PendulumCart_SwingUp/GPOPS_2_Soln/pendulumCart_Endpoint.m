function output = pendulumCart_Endpoint(input)

%The cost function is the integral of force.^2
output.objective = input.phase(1).integral;

end