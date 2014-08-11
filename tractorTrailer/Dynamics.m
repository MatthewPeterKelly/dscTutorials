function dStates = Dynamics(States, Inputs, Params)

%This function is used to compute the dynamics of the tractor trailer truck
%system. The equations are derived by Derive_EoM.m

%States is a (4xN) matrix of states
%Inputs is a (2xN) matrix of inputs
%Params is a structure of parameters

%x = States(1,:);
%y = States(2,:);
th = States(3,:);
phi = States(4,:);

v = Inputs(1,:);
psi = Inputs(2,:)';

Lt = Params.Dyn.Lt;
Lc = Params.Dyn.Lc;

%[dStates, A, B] = Derive_EoM()

dStates =[...
 
                      -v.*cos(phi).*cos(psi).*sin(th);
                      v.*cos(phi).*cos(psi).*cos(th);
                         (v.*cos(psi).*sin(phi))/Lt;
 (v.*(Lt.*sin(psi) - Lc.*cos(psi).*sin(phi)))/(Lc*Lt)];


end