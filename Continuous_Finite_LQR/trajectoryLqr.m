function Soln = trajectoryLqr(t,linSys,Q,R,F,tol)

% This function is used to solve the finite-horizon continuout-time linear
% quadratic regulator problem.

% [A, B] = linSys(t)

nState = size(Q,1);
nInput = size(R,1);

userFun = @(t,z)rhs(t,z,linSys,Q,R,nState);
z0 = reshape(F,nState*nState,1);
tSpan = [t(end),t(1)];

options = odeset();
options.RelTol = tol;
options.AbsTol = tol;
sol = ode45(userFun,tSpan,z0);
z = deval(sol,t);

nSoln = length(t);
Soln(nSoln).t = 0;
Soln(nSoln).K = zeros(nState,nInput);
Soln(nSoln).S = zeros(nState,nState);
Soln(nSoln).E = zeros(nState,1);

for idx=1:nSoln
    i = nSoln-idx+1;
    zNow = z(:,i);
    tNow = t(i);
    S = reshape(zNow,nState,nState);
    [A,B] = linSys(tNow);
    K = R\(B'*S);
    Soln(i).t = tNow;
    Soln(i).K = K;
    Soln(i).S = S;
    Soln(i).E = eig(A-B*K);
end

end

function dz = rhs(t,z,linSys,Q,R,nState)
P = reshape(z,nState,nState);
[A,B] = linSys(t);
dP = ricatti(A,B,Q,R,P);
dz = reshape(dP,nState*nState,1);
end

function [dP, K] = ricatti(A,B,Q,R,P)
K = R\B'*P;
dP = -(A'*P + P*A - P*B*K + Q);
end