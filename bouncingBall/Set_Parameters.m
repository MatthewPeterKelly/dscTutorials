function P = Set_Parameters()

%This function is used to collect all of the user-specified parameters and
%return them as a single struct, P.

%% Physical parameters
    P.gravity = 9.81;    %(m/s^2) gravitational acceleration
    P.mass = 0.3;        %(kg) mass of the ball
    P.drag = 0.02;       %(N*s^2/m^2) quadratic drag coefficient

    %NOTE - if the drag is set to negative, then the speed will
    %exponentially increase throughout the simulation, eventually  reaching
    %infinity. This is not suggested.
    if P.drag < 0
        disp('WARNING - air drag is negative - not advised!');
    end
    
    %It is assumed that the collision occurs instantaneously, and that the
    %coefficient of restitution is applied normal to the ground at the
    %point of impact. The tangential component of the velocity is
    %unaffected by the collision. 
    P.coeff_restitution = 0.95;  
    
    %If the ratio between the tangential and normal components of the ball
    %after the collision is larger than this number, then assume that the
    %ball will start rolling. Note- normal and tangential components are
    %measured with respect to the collision surface.
    P.rollingThreshold = 1e3;

%% Exit conditions
% Run until either the maximum number of bounces or the maximum time 
    P.maxBounce = 6;  %Number of impact events before exit
    P.maxTime = 25;   %Maximum simulation time


%% Initial Conditions:

    Pos_X = 0;    %(m) Initial horizontal position of the ball
    Pos_Y = 1.5;  %(m) Initial vertical position of the ball
    Vel_X = 0;    %(m/s) Initial horizontal speed of the ball
    Vel_Y = -1;   %(m/s) Initial vertical speed of the ball

    P.initCond = [Pos_X; Pos_Y; Vel_X; Vel_Y];


%% ode45 option struct
    P.Options = odeset(  'RelTol',1e-6,...
                        'AbsTol', 1e-6,...
                        'Events', @EventFunction,...
                        'Vectorized',true,...
                        'MaxStep',0.1);    
          %NOTE: We should not need to set MaxStep, but it seems that the
          %event detection in ode45 misses some events when it takes large
          %time steps, if the system dynamics are simple, as is the case
          %here, especially when there is negligable drag.
                
%% Display parameters (plotting + animation)

    %ode45 returns points that are widely spaced and not uniformly spaced.
    %Thus, for better plotting I use the function deval to accurately
    %interpolate the solution to a more useful grid spacing, while
    %preserving the endpoints of each trajectory.
    P.plotTimeStep = 0.005;


    %Sometimes it is useful to run the animation either faster or slower
    %than real time. The simulation time tracks the current cpu time,
    %divided by this number. Thus a value of 2 means play a 1 second
    %simulation over the course of 2 seconds of cpu time.
    P.slowMotionFactor = 1;
    
    %Set the font and line sizes for the figures:
    P.TitleFontSize = 18;
    P.LabelFontSize = 16;
    P.LegendFontSize = 14;
    P.AxisFontSize = 14;
    P.CurveLineWidth = 3;
    P.PlotBallSize = 50;

end