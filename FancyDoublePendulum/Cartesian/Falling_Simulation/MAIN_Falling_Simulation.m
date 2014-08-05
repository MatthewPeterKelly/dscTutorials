%MAIN_Falling_Simulation
%
% Test the mechanics equations by simulating the model falling while
% switching phases. All actuators are controlled by simple PD controllers,
% which may have gains set to zero.
%
% Equations derived with function: Derive_EoM.m
%
% Matthew Kelly
% December 7, 2013
% Cornell
%

clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Simulation Parameters                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% These parameters are for the physics simulation (not visualization)

P.Sim.nPhases = 25;   %Cycle through 10 random phases
P.Sim.phaseDuration = [0.8,1.9];   %Bounds on each phase's length
P.Sim.options = odeset('RelTol',1e-8,'AbsTol',1e-8);

P.Sim.springDoublePendulum = true;  
    %true = set controller to act as springs (conserves energy)
    %false = use a generic PD controller (does not conserve energy)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Animation Parameters                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% These parameters are for the animation routine (visualization only)

P.Animation.timeRate = 0.8;  %How fast does simulation time pass?
P.Animation.zoomScale = 3;   %1 = close fit, >1 = zoom out
P.Animation.tracking = 0.4;  %0 = jump to next, 1 = continuous tracking

P.Animation.save = false;    %Save the animation to file?
P.Animation.frameRate = 30;   %(frames/s) Only used when saving animation

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                   Plotting Parameters Parameters                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%These parameters are for static plots (visualization only)
P.Plotting.nPoints = 1000;   %Number of points per curve.


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Configure Control                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
P.Control = setupController(P);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Model Parameters                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.Dyn.m1 = 1;  %(kg)
P.Dyn.m2 = 1;  %(kg)
P.Dyn.M = 1;   %(kg)
P.Dyn.g = 9.81;  %(m/s^2)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Initial Contitions                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

S_init.x0 = 0; % (m) Hip horizontal position
S_init.y0 = 0; % (m) Hip vertical position
S_init.x1 = -1; % (m) Foot One horizontal position
S_init.y1 = 0; % (m) Foot One vertical position
S_init.x2 = 1; % (m) Foot Two horizontal position
S_init.y2 = 0; % (m) Foot Two vertical position
S_init.dx0 = 0; % (m) Hip horizontal velocity
S_init.dy0 = 0; % (m) Hip vertical velocity
S_init.dx1 = 0; % (m) Foot One horizontal velocity
S_init.dy1 = 0; % (m) Foot One vertical velocity
S_init.dx2 = 0; % (m) Foot Two horizontal velocity
S_init.dy2 = 0; % (m) Foot Two vertical velocity

State_init = convert(S_init);   %Convert struct to a matrix

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Get Phase Schedule                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

Modes = {'F','D','S1','S2'};
SimDat = cell(P.Sim.nPhases,1);
for i=1:length(SimDat)
    SimDat{i}.phase = Modes{randi(length(Modes))};
    bndT = P.Sim.phaseDuration;
    SimDat{i}.duration = bndT(1) + diff(bndT)*rand(1);
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Simulation                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
tLast = 0;
for idxPh=1:length(SimDat)
    if idxPh > 1
        PhaseBefore = SimDat{idxPh-1}.phase;
        PhaseAfter = SimDat{idxPh}.phase;
        State_init = phaseMap(State_final', PhaseBefore, PhaseAfter);
    end
    Tspan = [tLast,tLast + SimDat{idxPh}.duration];
    userFunc = @(t,z)rhs(t,z,SimDat{idxPh}.phase,P);
    sol = ode45(userFunc,Tspan,State_init,P.Sim.options);
    State_final = sol.y(:,end);
    SimDat{idxPh}.sol = sol;
    tLast = sol.x(end);
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                                Plots                                    %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

energyPlotHandle = figure(1); clf; 
plotEnergy(SimDat,P);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Animation (Real Time)                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if ~P.Animation.save
    timeRate = P.Animation.timeRate;
    figure(2);
    plotFrame();  %Reset the frame data
    idxPh = 1;
    tic
    timeNow = toc*timeRate;
    while idxPh <= length(SimDat)
        if timeNow > SimDat{idxPh}.sol.x(end)
            idxPh = idxPh + 1;
        else
            plotFrame(SimDat{idxPh},timeNow,P);
            timeNow = toc*timeRate;
        end
    end
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Animation (Render to File)                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if P.Animation.save
    save2pdf('savedEnergyPlot.pdf',energyPlotHandle,400);
    figure(2);
    plotFrame();  %Reset the plotting data
    Tspan = [SimDat{1}.sol.x(1), SimDat{end}.sol.x(end)];
    nFrames = diff(Tspan)*P.Animation.frameRate;
    time = linspace(Tspan(1),Tspan(2),nFrames);
    idxPh = 1;
    for i=1:nFrames
        if time(i) > SimDat{idxPh}.sol.x(end)
            idxPh = idxPh + 1;
        end
        plotFrame(SimDat{idxPh},time(i),P);
        MovieData(i) = getframe(gcf);
    end

    videoObject = VideoWriter('savedAnimation','MPEG-4');
    videoObject.FrameRate = P.Animation.frameRate;
    videoObject.open();
    writeVideo(videoObject,MovieData);
    videoObject.close();
end







