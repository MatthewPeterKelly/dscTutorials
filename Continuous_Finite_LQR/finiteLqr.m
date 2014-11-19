function Soln = finiteLqr(tSpan,A,B,Q,R,F,nSoln,tol)

%This function solves the finite-horizon continuous-time linear quadratic
%regulator problem for a linear plant.

nState = size(A,1);
nInput = size(B,2);

userFun = @(t,z)rhs(t,z,A,B,Q,R,nState);
z0 = reshape(F,nState*nState,1);
tSpan = fliplr(tSpan);  %Integrate backwards in time

options = odeset();
options.RelTol = tol;
options.AbsTol = tol;
sol = ode45(userFun,tSpan,z0);
t = linspace(tSpan(2),tSpan(1),nSoln);
z = deval(sol,t);

Soln(nSoln).t = 0;
Soln(nSoln).K = zeros(nState,nInput);
Soln(nSoln).S = zeros(nState,nState);
Soln(nSoln).E = zeros(nState,1);

for i=1:nSoln
    zNow = z(:,i);
    tNow = t(i);
    S = reshape(zNow,nState,nState);
    K = R\(B'*S);
    Soln(i).t = tNow;
    Soln(i).K = K;
    Soln(i).S = S;
    Soln(i).E = eig(A-B*K);
end

end

function dz = rhs(~,z,A,B,Q,R,nState)
P = reshape(z,nState,nState);
dP = ricatti(A,B,Q,R,P);
dz = reshape(dP,nState*nState,1);
end

function [dP, K] = ricatti(A,B,Q,R,P)
K = R\B'*P;
dP = -(A'*P + P*A - P*B*K + Q);
end