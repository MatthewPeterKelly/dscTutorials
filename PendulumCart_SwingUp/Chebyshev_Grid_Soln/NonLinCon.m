function [C, Ceq] = NonLinCon(x,P)

%This function is used to compute the nonlinear constraints for the
%pendulum cart swing-up problem. In this case, all constraints are defect
%constraints on the state.


%Expand the decision variable vector to a struct
X = convertVecStruct(x,P.structDat);

%Get the full state matrix (but the starting and ending states back on)
States = [P.MS.Start, X.state, P.MS.Finish];

%Get the state derivatives using the differentiation matrix:
dStates_Cheby = chebyshevDerivative(States,[0,X.duration]);

%Get the state derivatives using the system dynamics:
dynFunc = P.Dyn.dynamics_func;
dStates_Dyn = feval(dynFunc,States,X.force,P.Dyn);

%Compute the defect matrix:
defects = dStates_Cheby - dStates_Dyn;

%Format things nicely for returning to fmincon:
[n,m] = size(defects);
Ceq = reshape(defects,n*m,1);
C = [];

end