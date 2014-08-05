function Results = wrapper_SNOPT(P,X,Xlow,Xupp)

global PARAMETERS;

%WRITE to global variable
PARAMETERS = P;

x = convertVecStruct(X,P.structDat);
xlow = convertVecStruct(Xlow,P.structDat);
xupp = convertVecStruct(Xupp,P.structDat);

F = snoptObjective(x);

%For now, we only have equality constraints
Flow = zeros(size(F));
Fupp = Flow;

%The first element is the cost, which is unbounded:
Flow(1) = -inf;
Fupp(end) = inf;

%Set accuracy for SNOPT
snsetr('Major feasibility tolerance',P.options.TolFun)
snsetr('Major optimality tolerance',P.options.TolFun)

userfun = 'snoptObjective';

%Run snopt:
tic
  [xSoln,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,userfun);
cpuTime = toc;

%Get the solution struct:
Xsoln = convertVecStruct(xSoln,P.structDat);

%Store the solver output:
name = 'snopt';
Fval = F(1);
input = makeStruct(name,xSoln,xlow,xupp,Flow,Fupp,userfun);
output = makeStruct(x,F,Fval,inform,xmul,Fmul, cpuTime);
Results = makeStruct(P,input,output,Xsoln);

end