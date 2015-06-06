function [problem, dim] = buildConstraints(guess,config)

% Pack up guess:
tSpan = guess.time([1,end]);
[zGuess,dim] = packDecVar(tSpan,guess.state,guess.control);
nDecVar = length(zGuess);

% Get indicies for each decision variable
[tIdx,xIdx,uIdx] = getIndex(dim);

% Store bounds for each decision variable:
lb = zeros(nDecVar,1);
ub = zeros(nDecVar,1);
B = config.bounds;

lb(tIdx(1)) = B.initialTime.lower;
ub(tIdx(1)) = B.initialTime.upper;

lb(tIdx(2)) = B.finalTime.lower;
ub(tIdx(2)) = B.finalTime.upper;

lb(xIdx(:,1)) = B.initialState.lower;
ub(xIdx(:,1)) = B.initialState.upper;

lb(xIdx(:,end)) = B.finalState.lower;
ub(xIdx(:,end)) = B.finalState.upper;

ns = dim.nState(2);
for i=2:(ns-1)
    lb(xIdx(:,i)) = B.state.lower;
    ub(xIdx(:,i)) = B.state.upper;
end

nc = dim.nControl(2);
for i=1:nc
    lb(uIdx(:,i)) = B.control.lower;
    ub(uIdx(:,i)) = B.control.upper;
end

% Figure out which are equality (vs inequality) constraints:
idxEq = lb==ub;
nEq = sum(idxEq);
index = 1:nDecVar;
nIneq = sum(~idxEq);

% Equality constraints
Aeq = zeros(nEq,nDecVar);
beq = lb(idxEq);   %List of equality constraints:
eqIdx = index(idxEq);  %List of decision variable indicies
for i=1:nEq
   Aeq(i,eqIdx(i)) = 1;
end

% Inequality constraints
Aineq = zeros(2*nIneq,nDecVar);
bineq = zeros(2*nIneq,1);
low = lb(~idxEq);
upp = ub(~idxEq);
ineqIdx = index(~idxEq);
for i=1:nIneq
    idx = 2*(i-1);
   Aineq(idx+1,ineqIdx(i)) = 1; 
   Aineq(idx+2,ineqIdx(i)) = 1; 
      bineq(idx+1) = low(i); 
   bineq(idx+2) = upp(i); 
end

% Build the problem struct:
problem.x0 = zGuess;
problem.Aineq = sparse(Aineq);
problem.bineq = bineq;
problem.Aeq = sparse(Aeq);
problem.beq = beq;
problem.lb = [];
problem.ub = [];
problem.options = config.options.nlp;
problem.solver = 'fmincon';

end