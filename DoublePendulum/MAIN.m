%MAIN  --  Run Simulation of Double Pendulum

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% 
%%%% Written by Matthew Kelly
%%%% October 12, 2013
%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Set the parameters for the model and derive the dynamics
P = Set_Parameters();

%U is a vector of torque and force inputs on the system. For now, simulate
%the unforced system.
U = zeros(6,1);

%What kind of dynamics function?
if P.misc.CppFlag
    userFunc = @(t,X)Evaluate_Dynamics(X,U,P.CppVec);
else
    userFunc = @(t,X)Double_Pendulum_Dynamics(t,X,U,P.dyn);
end

%Initialize the integration
X0 = P.sim.InitState;
Tspan = P.sim.Tspan;

%Run integration
disp('Running simulation...')
OdeOpt = P.sim.OdeOpt;
[tout, xout] = ode45(userFunc,Tspan,X0,OdeOpt);

%Compute kinematics
[K, E] = Double_Pendulum_Kinematics(xout',P.dyn,U); 

%Animation:
Animate_Double_Pendulum(tout,K,P)

%Plotting
 Plot_Double_Pendulum(tout,xout,E,P)