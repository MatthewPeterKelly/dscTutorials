function [P, X] = Set_Parameters()
%[P, X] = Set_Parameters()
%
% FUNCTION:
%   This function returns all user-sepcified parameters as a struct and the
%   initial guess for the decision variables in the optimization.
%
% INPUTS: 
%   
% OUTPUTS:
%   P = A struct with a field for each set of parameters
%   X = A struct that contains the initial guess for decision variables
%

%% Parameters for multiple shooting

    P.MS.nGrid = 100;    %Number of grid points to use

    P.MS.Start = [0;0;pi;0]; %Initial state [x,v,th,w];
    P.MS.Finish = [0;0;0;0];  %Final state [x,v,th,w];

    %Attempt interpolate from a saved solution:
    P.MS.Initialize_from_File = true;
    
    %Pick the solver:
    %P.MS.solver = 'snopt';   %  'fmincon' OR 'snopt'
    P.MS.solver = 'fmincon';   %  'fmincon' OR 'snopt'
    
    %Cost function:
    P.MS.cost_function = 1;
    %   0 = none          
    %   1 = force squared
    
    
%% FMINCON options  (Tolis also used in SNOPT)
    MaximumIterations = 400;
    Tol = 1e-12;
    P.options = optimset(...
        'Algorithm', 'interior-point',...  %  'interior-point'  'sqp'  'active-set'
        'Display', 'iter-detailed',...
        'MaxIter', MaximumIterations,...
        'MaxFunEvals',MaximumIterations*(P.MS.nGrid*5+1),...  %Equivilent number of function evaluations
        'TolFun', Tol,...
        'TolX',1e-16,...
        'TolCon',Tol);
    
    
%% Bounds on the decision variables
% Note - the state and actuator bounds are only enforced at the grid
% points. It is possible for the actual values of the solution to exceed
% these bounds.

    P.Bnd.force = 100*[-1,1];  %Bounds on the horizontal actuator

    P.Bnd.state = [...
        -2, 2;     %Bounds on horizontal position of the cart
        -10,10;    %Bouds on the horizontal velocity of the cart
        -3*pi, 3*pi;  %Bounds on the pendulum angle
        -2*2*pi, 2*2*pi];   %Bounds on the pendulum angular velocity

    P.Bnd.duration = [0.01,2];   


%% Parameters for dynamics

    P.Dyn.m = 0.8;   %Mass of the pendulum
    P.Dyn.M = 1;   %Mass of the cart
    P.Dyn.g = 9.81;   %Set negative to have 0 be the unstable equilibrium
    P.Dyn.L = 0.3;   %Length of the pendulum

    P.Dyn.dynamics_func = @(x,f)Pendulum_Cart_Dynamics_Forced(x,f,P.Dyn);
    

%% Get the initial state:
    X = Initial_State(P);
    
    %Get the names and dimensions of each field in X
    [~,P.structDat] = convertVecStruct(X);
      
    
    
end
