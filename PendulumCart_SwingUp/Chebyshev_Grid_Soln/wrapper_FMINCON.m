function Results = wrapper_FMINCON(P,X,Xlow,Xupp)

%Function Handles:
objfunc = @(x)Objective_Function(x,P);
nonlcon = @(x)NonLinCon(x,P);

%No linear constraints:
A=[]; Aeq=[]; B=[]; Beq=[];
          
%Store things in a Problem struct:
  Problem.objective = objfunc;
  Problem.x0 = convertVecStruct(X,P.structDat);
  Problem.Aineq = A;
  Problem.bineq = B;
  Problem.Aeq = Aeq;
  Problem.beq = Beq;
  Problem.lb = convertVecStruct(Xlow,P.structDat);
  Problem.ub = convertVecStruct(Xupp,P.structDat);
  Problem.nonlcon = nonlcon;
  Problem.options = P.options;
  Problem.solver = 'fmincon';
  
%Run fmincon:
tic
  [xSoln, Fval, ExitFlag, Output] = fmincon(Problem);
cpuTime = toc;

%Get the solution struct:
Xsoln = convertVecStruct(xSoln,P.structDat);

%Store the solver output:
name = 'fmincon';
input = makeStruct(name, Problem);
output = makeStruct(xSoln, Fval, ExitFlag, Output, cpuTime);
Results = makeStruct(P,input,output,Xsoln);

end