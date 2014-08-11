%MAIN_fsm

clc; clear;

%%%% Derive equations of motion %%%%
% EoM_flight();
% EoM_slide();
% EoM_hinge();

%%%% Interesting Parameters %%%%
MoI = 0.333;  %Dimensionless moment of inertia
    % 0   => Point mass at center of rod
    % 1/3 => Slender rod (uniform distribution)
    % 1   => Point mass at both ends of the rod
CoeffFriction = 0.2;
%%%% ~~~~~~~~~~~~~~~~~~~~~~ %%%%

P.g = 9.81;
P.L = 1;  %Distance between END OF STICK AND CENTER OF MASS
P.FullLength = 2*P.L;   %Length of the entire stick
P.m = 1;
P.u = CoeffFriction;   %Coefficient of friction
P.I = MoI*P.m*P.L^2;  %Theoretical Maximum - two point masses
setup.P = P;

setup.IC.th = 1e-2;
setup.IC.x = 0;
setup.IC.y = 0;
setup.IC.dth = topple_angularRate(setup.IC.th,P);
setup.IC.dx = 0;
setup.IC.dy = 0;

setup.Tspan = [0,10];  %only used for timeout of the simulation
setup.tol = 1e-12;   %Accuracy of the intergation method
setup.dataFreq = 750;   %How much data to return?
setup.solver = @ode45; %Integration method
setup.odeMaxStep = 1e-1;  %Biggest allowable step

D = runSimulation(setup);

plotData(D);

for i=1:length(D.phase)
    disp([D.phase{i} '  ->  ' D.code{i}])
end

timeRate = 0.5;   % slow motion < 1 < fast forward
animation(D,timeRate);

