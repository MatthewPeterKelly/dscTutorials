function [C, Ceq] = boundaryConstraint(t0,tF,x0,xF,userData)

C = [];
Ceq = [
    t0; % Start at t==0
    x0 - userData.initialState;  %Initial state  
    xF-userData.finalState];   %Final state

end