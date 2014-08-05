function F = snoptObjective(x)

global PARAMETERS;

%READ from global variable
P = PARAMETERS;

cost = Objective_Function(x,P);

[C, Ceq] = NonLinCon(x,P);

if ~isempty(C)
  error('snoptObjective cannot deal with inequality constraints.')
end

F = [cost;Ceq];

end