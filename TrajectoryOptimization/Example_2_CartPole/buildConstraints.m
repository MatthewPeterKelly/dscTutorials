function [problem, pack] = buildConstraints(guess,config)

% Pack up guess:
[zGuess,pack] = packDecVar(guess.time(end),guess.state,guess.control);
nDecVar = length(zGuess);

%%%% Build linear constraints:
[tIdx,xIdx,uIdx] = getIndex(pack);

% State and control bounds:  (bounds on input)
lb = zeros(nDecVar,1);
ub = zeros(nDecVar,1);
lb(tIdx) = config.bounds.duration.lower;
ub(tIdx) = config.bounds.duration.upper;
for i=1:4
lb(xIdx.x(i,:)) = config.bounds.state.lower(i);
ub(xIdx.x(i,:)) = config.bounds.state.upper(i);
end
lb(uIdx.u) = config.bounds.control.lower(1);
ub(uIdx.u) = config.bounds.control.upper(1);

% Build the problem struct:
problem.x0 = zGuess;
problem.Aineq = [];
problem.bineq = [];
problem.Aeq = [];
problem.beq = [];
problem.lb = lb;
problem.ub = ub;
problem.options = config.options.nlp;
problem.solver = 'fmincon';

end