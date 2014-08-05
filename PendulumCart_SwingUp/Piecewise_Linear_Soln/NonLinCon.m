function [C, Ceq] = NonLinCon(x,P)

%This function is used to compute the nonlinear constraints for the
%pendulum cart swing-up problem. In this case, all constraints are defect
%constraints on the state.

%Expand the decision variable vector to a struct
X = convertVecStruct(x,P.structDat);

%Get the full state matrix (but the starting and ending states back on)
States = [P.MS.Start, X.state, P.MS.Finish];

%Set up for integration using RK4, include cost in integral
Ylow = States(:,1:(end-1));
Yupp = States(:,2:end);

Ulow = X.force(1:(end-1));
Uupp = X.force(2:end);
Umid = 0.5*(Ulow + Uupp);


userFunc = P.Dyn.dynamics_func;  %ARG(state,force);
h = X.duration/(P.MS.nGrid-1);
 
%Runge-Kutta 4th order
k1 = feval(userFunc,Ylow,Ulow);
k2 = feval(userFunc,Ylow + 0.5*h*k1,Umid);
k3 = feval(userFunc,Ylow + 0.5*h*k2,Umid);
k4 = feval(userFunc,Ylow + h*k3,Uupp);
Ynew = Ylow + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

%Compute the defects:
defects = Ynew-Yupp;

%Format things nicely for returning to fmincon:
[n,m] = size(defects);
Ceq = reshape(defects,n*m,1);
C = [];

end