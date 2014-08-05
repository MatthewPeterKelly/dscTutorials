function Cost = Objective_Function(x,P)

%The cost function is actually computed in PhysicsIntegration. Since the
%cost functions for this type of problem are time-integrals, it makes sense
%to use the same integration for the system dynamics as for the cost
%function. If there was also a discrete cost, or other non-integral cost,
%it would be added in this function.

Soln = PhysicsIntegration(x,P);
Cost = Soln.Total_Cost;

end






