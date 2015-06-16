%---------------------- MAIN.m -- Double Pendulum Walker -----------------%
% This code finds the optimal trajectory for a double pendulum model of   %
% a biped to take one walking step over flat ground.                      %
%-------------------------------------------------------------------------%

clc; clear;

%-------------------------------------------------------------------------%
%------------------------ Model Parameters -------------------------------%
%-------------------------------------------------------------------------%

gravity = 9.81;     %(m/s^2) gravitational acceleration
legLength = 0.96;   %(m) leg length of the model walker
hipMass = 9.21;     %(kg) hip mass for the model walker
footMass = 0.71;    %(kg) foot mass for the modell walker

auxdata.dynamics.g = gravity;
auxdata.dynamics.L = legLength;
auxdata.dynamics.m1 = hipMass;
auxdata.dynamics.m2 = footMass;


%-------------------------------------------------------------------------%
%------------------------ Other Parameters -------------------------------%
%-------------------------------------------------------------------------%

auxdata.cost.model = 'TorqueSquared'; %{'TorqueSquared', 'CostOfTransport'}

auxdata.cost.workSmoothing = 1e-1;
auxdata.cost.negativeWork = 0.5;   %how much to count negative work:
    %  1 = full cost (same as positive work)
    %  0 = no cost (ignored)
    % -1 = full regeneration

%-------------------------------------------------------------------------%
%------------------------ Index Definitions ------------------------------%
%-------------------------------------------------------------------------%

idx.NUMBER_OF_STATES = 4;
    idx.THETA = 1;    %Absolute angle of the stance leg
    idx.PHI = 2;      %Absolute angle of the swing leg
    idx.DTHETA = 3;   %Absolute rate of the stance leg 
    idx.DPHI = 4;     %Absolute rate of the swing leg

idx.NUMBER_OF_CONTROLS = 2;
    idx.HIP = 1; %Torque from stance leg on swing leg 
    idx.ANK = 2; %Torque from ground on stance leg

idx.NUMBER_OF_DISTURBANCES = 4;
    idx.DIST_FORCE_HIP_X = 3;
    idx.DIST_FORCE_HIP_Y = 4;
    idx.DIST_FORCE_ANK_X = 5;
    idx.DIST_FORCE_ANK_Y = 6;
 
    
%-------------------------------------------------------------------------%
%------------------------ Time Bounds ------------------------------------%
%-------------------------------------------------------------------------%
iphase = 1;
bounds.phase(iphase).initialtime.lower = 0;
bounds.phase(iphase).initialtime.upper = 0;
bounds.phase(iphase).finaltime.lower = 0.2;
bounds.phase(iphase).finaltime.upper = 2;

%-------------------------------------------------------------------------%
%------------------------ State Bounds -----------------------------------%
%-------------------------------------------------------------------------%
iphase = 1;

Bnd_State_Low = zeros(1,idx.NUMBER_OF_STATES);
Bnd_State_Low(idx.THETA) = (pi/180)*30;         %(rad)
Bnd_State_Low(idx.PHI) = (pi/180)*(60-180);     %(rad/s)
Bnd_State_Low(idx.DTHETA) = -3*pi;              %(rad)
Bnd_State_Low(idx.DPHI) = -3*pi;                %(rad/s)

Bnd_State_Upp = zeros(1,idx.NUMBER_OF_STATES);
Bnd_State_Upp(idx.THETA) = (pi/180)*(180-30);   %(rad)
Bnd_State_Upp(idx.PHI) = (pi/180)*(0-60);       %(rad/s)
Bnd_State_Upp(idx.DTHETA) = 3*pi;               %(rad)
Bnd_State_Upp(idx.DPHI) = 3*pi;                 %(rad/s)

bounds.phase(iphase).initialstate.lower = Bnd_State_Low;
bounds.phase(iphase).initialstate.upper = Bnd_State_Upp;
bounds.phase(iphase).state.lower = Bnd_State_Low;
bounds.phase(iphase).state.upper = Bnd_State_Upp;
bounds.phase(iphase).finalstate.lower = Bnd_State_Low;
bounds.phase(iphase).finalstate.upper = Bnd_State_Upp;


%-------------------------------------------------------------------------%
%------------------------ Control Bounds ---------------------------------%
%-------------------------------------------------------------------------%
iphase = 1;

bounds.phase(iphase).control.lower = zeros(1,idx.NUMBER_OF_CONTROLS);
bounds.phase(iphase).control.lower(idx.HIP) = -1;    %(Nm)
bounds.phase(iphase).control.lower(idx.ANK) = 0.0; %%%%HACK -0.7;  %(Nm)

bounds.phase(iphase).control.upper = zeros(1,idx.NUMBER_OF_CONTROLS);
bounds.phase(iphase).control.upper(idx.HIP) = 1;     %(Nm)
bounds.phase(iphase).control.upper(idx.ANK) = 0.0; %%%%HACK 0.7;   %(Nm)

    %How to weight cost for impulse and continuous?
    %   -> Assume that the impulse is equal to a constant force over a
    %   finite time. Then, Assume that the force is equal to a constant
    %   motor torque applied over some constant lever arm. Result:
    %   -> Cost = (Impulse Squared)*(Lever Arm Squared) / Duration;
    Impulse_Duration = 0.025;   %(s) 25 millisecond impulse duration
    Impulse_LeverArm = 0.1;  %(m) Fraction of leg length
    Impulse_MaxTorque = 0.7;   %(Nm)
    MaxImpulse = Impulse_MaxTorque*Impulse_Duration/Impulse_LeverArm;
    ImpulseConstant = Impulse_LeverArm^2/(Impulse_Duration);
    
%The only parameter is the pushoff impulse magnitude
bounds.parameter.lower = 0;
bounds.parameter.upper =  0.0; %%%%HACK MaxImpulse;
guess.parameter = 0.5*MaxImpulse;
auxdata.cost.ImpulseConstant = ImpulseConstant; 
    %Multiply this by impulse^2 to get compatible units for a
    %torque-squared cost function

%-------------------------------------------------------------------------%
%------------------------ Cost Integral ----------------------------------%
%-------------------------------------------------------------------------%
iphase = 1;

%The integral is used in the continuous cost function
switch auxdata.cost.model
    case 'TorqueSquared'
        Bnd_Integral_Cost_Hip = [0,10];
        Bnd_Integral_Cost_Ank = [0,10];
    case 'CostOfTransport'
        Bnd_Integral_Cost_Hip = [0,10];
        Bnd_Integral_Cost_Ank = [0,10];
    otherwise
        error('Invalid Cost Model')
end

bounds.phase(iphase).integral.lower = zeros(1,idx.NUMBER_OF_CONTROLS);
bounds.phase(iphase).integral.upper = zeros(1,idx.NUMBER_OF_CONTROLS);

bounds.phase(iphase).integral.lower(idx.HIP) = Bnd_Integral_Cost_Hip(1);
bounds.phase(iphase).integral.upper(idx.HIP) = Bnd_Integral_Cost_Hip(2);

bounds.phase(iphase).integral.lower(idx.ANK) = Bnd_Integral_Cost_Ank(1);
bounds.phase(iphase).integral.upper(idx.ANK) = Bnd_Integral_Cost_Ank(2);

guess.phase(iphase).integral(idx.HIP) = mean(Bnd_Integral_Cost_Hip); 
guess.phase(iphase).integral(idx.ANK) = mean(Bnd_Integral_Cost_Ank); 

%-------------------------------------------------------------------------%
%------------------------ Initialization ---------------------------------%
%-------------------------------------------------------------------------%
iphase = 1;
bnd = bounds.phase(iphase);

t0 = 0.5*(bnd.initialtime.lower + bnd.initialtime.upper);
t1 = 0.5*(bnd.finaltime.lower + bnd.finaltime.upper);
guess.phase(iphase).time = [t0; t1];

%For now, initialize as constant in middle of bounds
xMean = 0.5*(bnd.state.lower + bnd.state.upper);
guess.phase(iphase).state = [xMean; xMean];

%For now, initialize as constant in middle of bounds
uMean = 0.5*(bnd.control.lower + bnd.control.upper);
guess.phase(iphase).control = [uMean; uMean];


%-------------------------------------------------------------------------%
%------------------------------ Constraints ------------------------------%
%-------------------------------------------------------------------------%

%Periodic constraint from state after heel-strike to initial state
idx.cst.PERIODIC = 1;
bounds.eventgroup(1).lower = zeros(1,idx.NUMBER_OF_STATES);
bounds.eventgroup(1).upper = zeros(1,idx.NUMBER_OF_STATES);

%Require the slope to be flat:
idx.cst.SLOPE= 2;
bounds.eventgroup(2).lower = 0;
bounds.eventgroup(2).upper = 0;

%Require a minimum step size:
idx.cst.STEP_LENGTH = 3;
bounds.eventgroup(3).lower = 0.3;   %(m)
bounds.eventgroup(3).upper = 0.7;   %(m)

%Velocity
idx.cst.VELOCITY = 4;
bounds.eventgroup(4).lower = 0.1;   %(m/s)
bounds.eventgroup(4).upper = 4.0;   %(m/s)

%-------------------------------------------------------------------------%
%----------- Assemble All Information into Setup Structure ---------------%
%-------------------------------------------------------------------------%

auxdata.index = idx; 

setup.name = 'Double-Pendulum-Walker';
setup.functions.continuous = @walkerContinuous;
setup.functions.endpoint = @walkerEndpoint;
setup.nlp.solver = 'ipopt';
setup.bounds = bounds;
setup.guess = guess;
setup.auxdata = auxdata;
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.derivatives.dependencies = 'sparseNaN';
setup.scales.method = 'automatic-bounds';
setup.mesh.method = 'hp-PattersonRao';
setup.mesh.tolerance = 1e-12;
setup.method = 'RPM-Integration';


%-------------------------------------------------------------------------%
%---------------------- Solve Problem using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
totaltime = tic;
output = gpops2(setup);
totaltime = toc(totaltime);

plotOutput(output);
