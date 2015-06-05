function [C, Ceq] = boundaryConstraint(t,x0,xF,userData)

C = [];
Ceq = [x0 - userData.initialState; xF-userData.finalState];

end