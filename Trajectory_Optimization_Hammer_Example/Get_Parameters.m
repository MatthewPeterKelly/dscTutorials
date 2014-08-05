function [P, X] = Get_Parameters()

%This function is used to define all parameters that a user can set. It
%returns a struct P with parameters, grouped into subfields, and a struct X
%with the initial guess at the state that is to be passed to fmincon.


%% Dynamics:
P.dyn.gravity = 9.81;  % (m/s^2)   Acceleration of gravity
P.dyn.length = 0.25;  % (m)   Length of Handle
P.dyn.mass = 0.2;   % (kg)  Hammer Point Mass
P.dyn.coeffRestitution = 0.2;  % () Coefficient of restitution


%% Optimization Algorithm:
% Type: optimset('fmincon') at command line to see all available options
P.opt = optimset(...
    'TolFun', 1e-8,...
    'TolX', 1e-12,...
    'TolCon', 1e-8,...
    'MaxIter',500,...
    'Display', 'iter-detailed',... % [ off | iter | iter-detailed | notify | notify-detailed | final | final-detailed ]
    'Algorithm', 'sqp',... % [ active-set | interior-point | interior-point-convex | levenberg-marquardt | sqp | trust-region-dogleg | trust-region-reflective ]
    'MaxFunEvals', 1e6,...
    'UseParallel', 'never'); % [ always | {never} ]


%% Discritization:
% This is the number of gridpoints along with trajectory
P.nGridPts = 20;


%% Cost Function
P.cost.Method = 0;
  % 0 = Time Integral of Torque^2 
  % 1 = Time Integral of Torque*Rate

  % IF P.cost.Method == 1
        P.cost.Count_Negative_Work = 0.2;  
            %Negative work is when the system is backdriving the motor.
             % 1 => negative work equal to positive work, 
             % 0 => ignore negative work,
             %-1 => full regeneration 
        P.cost.Motor_Cost_Smoothing = 1; 
            %It turns out that smoothing is very important for convergence.
            %Here I use a modified form hyperbolic tangent smoothing:
            % 0    ==>  Sharp
            % inf  ==>  Smooth
            % 0.1  -->  feasible, non-optimal, hit iteration limit
            % 0.5  -->  close, but with some numerical artifacts
            % 1.0  -->  finds correct solution after 121 iterations
            % 5.0  -->  heavy smoothing - correct solution after 158
            %           iterations. Note that the cost function increases
            %           nearly linearly. This is because the optimization
            %           has found the minimum value of the smoothed cost
            %           function, which is nearly flat at this point.
  % END
  
  
%% Initial Guess:
%Initialize the trajectory by guessing a few rough points.
  P.init.durationGuess = 1;
  P.init.angleGuess = (pi/180)*[90, 0, 90];
  P.init.rateGuess = 2*pi*linspace(-1, 2, length(P.init.angleGuess));
    
  %P.init.torqueGuess = linspace(-1, 1, length(P.init.angleGuess));    %Start with values in toruq
  P.init.torqueGuess = zeros(size(P.init.angleGuess));    %start with empty torque

%% Bounds:
%States
    P.bnd.angle = (pi/180)*[-25,91];   %90 = Table, 0 = Vertical
    P.bnd.rate = 2*pi*[-2,2];  
%Motor
    P.bnd.torque = [-1,1];  
%Duration
    P.bnd.duration = [0.8,1.2];

%% Display:
% As this optimization runs, you can choose to have it display it's
% progress towards the solution. There are also options to automatically 
% save the figures and generate the LaTeX code to put the figures into a
% document.

%Size the text on the figures nicely.
    P.disp.TitleFontSize = 16;
    P.disp.LabelFontSize = 16;
    P.disp.AxisFontSize = 12;
    P.disp.LineWidth = 3;
    P.disp.DefectLineWidth = 4;
    
%Decide if the plots should be shown as the code runs:
    P.disp.intermediatePlot = true;   %Use to toggle the plotting while running
    %IF P.disp.intermediatePlot, THEN
        P.disp.showIterationNum = ...%Creates intermediate plots at these iterations:
            [0,1,2,3,5,8,13,21,34,55,89,144,233,377,610,987];   
        P.disp.saveIntermediatePlots = false;   %Saves each auto-generated plot
        P.disp.createTex = true;   %Creates the skeleton of the LaTeX code to call figures
            P.disp.TexFileName = 'TexFigureCode.txt';
    %END
 
    
%% Constraints:
    P.cst.strikeAngle = pi/2;  %Hammer strikes exactly at this angle
    P.cst.strikeRate = max(P.bnd.rate);  %Reaches maximum speed at impact          


%% Automatic:
% This section of code consists of things that are automatically done by
% the code, and are not parameters to be adjusted by the user.

% Later on in the code, it is necessary to pass the 'state' (all of the
% decision variables in the optimization problem) as a vector. This format
% is difficuly to work with, so there is a function to convert between a
% vector and a struct. Part of the conversion requires knowledge of the
% structure of the state struct. Here we create the initial guess at the
% state, and then use this as a template to get the information about the
% structure of this struct and store is in P.structDat.
    X = Initial_State(P);   
    [~,P.structDat] = Convert_State(X); 

% PhysicsIntegration uses persistent variables to keep track of some
% things. Before running the optimization, we need to reset these
% variables. One way of doing this is to clear the function from memory.
% Another way (not used here) is to pass a flag to the function (such as
% calling it with no arguments) and it can then reset the persistent
% variables itself.
    clear PhysicsIntegration;   %Reset all of the persistent variables

% Matlab takes a bit of time to generate a function handle with extra
% arguments (in this case P), so we do it here to save some time. NOTE -
% this means that any future changes to P will not be passed to the
% dynamics function. This is fine here, but could produce problems if the
% code was written differently. 
    P.dyn.UserFunc = @(Z,u)HammerDynamics([],Z,u,P);

% This next block of code ensures that the LaTeX code fragmant starts with
% an empty file.
    if P.disp.createTex
        if exist(P.disp.TexFileName,'file')
            fid = fopen(P.disp.TexFileName,'w');
            fclose(fid);
        end
    end

end