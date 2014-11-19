%MAIN 
%
% Demonstrates how to solve the finite-horizon continuous-time linear
% quadratic regulator problem for a linear system
%

nState = 3;
nInput = 1;
nSoln = 100;

%LTI plant
A = 0.5*eye(nState) + 0.5*rand(nState);
B = rand(nState,nInput);

%LTI cost
Q = eye(nState);
R = eye(nInput);
F = zeros(nState);  %Terminal Cost

%Solve continuous LQR problem:
[K,S,E] = lqr(A,B,Q,R);

%Solve the finite-horizon version
tSpan = [0,30];  %Large time...
tol = 1e-6;
Soln = finiteLqr(tSpan,A,B,Q,R,F,nSoln,tol);

%Compare the results:  (Should go to zero for tSpan(2) goes to infinity)
K_error = K - Soln(1).K
S_error = S - Soln(1).S
E_error = E - Soln(1).E

%Make a plot showing what the gains look like:
figure(21); clf;
KK = reshape([Soln.K],nState,nSoln);
t = [Soln.t];
for i=1:nState
    subplot(nState,1,i); hold on;
   plot(tSpan,[K(i), K(i)],'k--','LineWidth',2) 
   plot(t,KK(i,:),'r-','LineWidth',2);
   ylabel(['K(' num2str(i) ')']);
end
xlabel('Time')

subplot(nState,1,1); 
title('Finite-Horizon Gains, Compare to Infinite Horizon Soln')

