function [C, Ceq] = NonLinCon(x,P)

%NonLinCon is used to get the non-linear constraints for the problem.
%Here, the only nonlinear cosntraints are the defects, which we get from
%the function: PhysicsIntegration. In more complicated problems there
%would be other constraints, which would be included in either C or Ceq.

Soln = PhysicsIntegration(x,P);
C = [];
Ceq = Soln.Defect_Vec;

end