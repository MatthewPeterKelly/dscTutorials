function [Xhat P] = Estimator(Z,Input,Params,Xhat,P)

%This function does a single step of an extended Kalman filter

%Unpack parameters
dt = Params.Sim.dt;
H = Params.Est.H;
R = Params.Est.R;
Q = Params.Est.Q;

dStates = Dynamics(Xhat, Input, Params);
Xbar = Xhat + dStates*dt;

%Linearize about Xhat
th = Xhat(3);
phi = Xhat(4);
v = Input(1);
psi = Input(2);
Lt = Params.Dyn.Lt;
%Lc = Params.Dyn.Lc;
F = A_Matrix(phi,th,psi,v,Lt);  
%G = B_Matrix(phi,th,psi,v,Lt,Lc);

%Propagate covariance
Pbar = F*P*F' + Q;

%Measurement Update
Nu = Z - H*Xbar;
S = H*Pbar*H' + R;
W = Pbar*H'/S;

%Estimates!
Xhat = Xbar + W*Nu;
P = Pbar - W*S*W';

end