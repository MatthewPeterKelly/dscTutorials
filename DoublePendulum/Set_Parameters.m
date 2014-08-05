function P = Set_Parameters()
%
% All options that are available to the user are entered using this
% function. It is broken up into sections, each of which corresponds to a
% single field in the parameter struct P.
%


%% Misc

    %Rederive the equations of motion?
    P.misc.DeriveMechanics = false;
    
    %Use the C++ version of the dynamics function
        % MEX must be configured to run with CppFlag=true
        % Type:  mex -setup  at the command line to configure 
    P.misc.CppFlag = false; 
    
%% Parameters for Dynamics  --  DO NOT CHANGE THE FIELD NAMES!
    P.dyn.g = 9.81;   %(m/s^2)
    P.dyn.L = 1;   %(m)    % Length of Ranger's leg
    P.dyn.m1 = 1;   %(kg)   Hip Mass  = (13/14) Ranger Mass
    P.dyn.m2 = 1;   %(Ns/m)  Foot Mass  =  (1/14) Ranger Mass
    
%% Simulation Stuff
    
    %Slow Motion Factor tells the simulation to run in slow motion:
    % 1 = Real Time
    % 2 = Slow Motion
    % 0.5 = Fast Forward
    P.sim.slow_motion_factor = 2;  %A positive real number
    
    %Initialize states
        th = (pi/180)*87;   %0 = x axis
        phi = (pi/180)*98;   %0 = x axis
        Dth = 0;
        Dphi = 0;
    P.sim.InitState = [th;phi;Dth;Dphi];
    
    %Set the time span
    P.sim.Tspan = [0,5];
    
    %Set the options
    P.sim.OdeOpt = odeset('RelTol',1e-6,'AbsTol',1e-6,'Vectorized',true);
    
%% Plotting and display options:

%Set the font and line sizes for the figures:
    P.plot.TitleFontSize = 16;
    P.plot.LabelFontSize = 14;
    P.plot.AxisFontSize = 14;
    P.plot.CurveLineWidth = 2;
    
%% Automatic Stuff

    if P.misc.DeriveMechanics
        %Derive the equations of motion and write them to a file
        disp('Deriving the continuous dynamics...')
        DeriveDynamicsDP(P);
        %The next two lines of code are confusing, but enable the user to
        %pass parameters to C++ in a way that minimizes copying errors by
        %automatically writing the parameter extraction header file. This
        %introduces a lot of overhead, but it is helpful once there are
        %control and estimation functions also in C++ that need parameters.
        Pcpp.dynamics = P.dyn;   %Store the parameters that go to C++
        P.CppVec = Write_Parameter_File(Pcpp);
    else
        %Ensure that the C++ parameters are passed correctly:
        Pcpp.dynamics = P.dyn;   %Store the parameters that go to C++
        P.CppVec = Flatten_Param_Struct(Pcpp);
        disp('Did not rederive equations - using existing code')
    end
    
    if P.misc.CppFlag
       mex Evaluate_Dynamics.cpp; 
       disp('Running with C++ dynamics function')
    else
        disp('Running with Matlab dynamics function')
    end
    
end