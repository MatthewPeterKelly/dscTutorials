function animation(plotInfo,figNum)

figure(figNum);

P = plotInfo.parameters;
Data = plotInfo.data;

timeRate = P.animation.timeRate;   %1 = Real time, 0.5 = slow motion, 2.0 = fast forward

%Check if we should save an animation
saveAnimation = false;
if isfield(P.animation,'fileData')
    saveAnimation = P.animation.fileData.save;
    frameRate = P.animation.fileData.frameRate;
end

%Creates an animation using plotInfo. This animation will be saved to a file
%if the struct fileData is included.

% Figure out how big to make the viewing window:
zoomScale = 1.1;  %How much to pad the viewing window (>=1);
rawExtents = getExtents(Data);
xMean = mean(rawExtents(1:2));
yMean = mean(rawExtents(3:4));
shiftExtents = [xMean, xMean, yMean, yMean];
P.animation.extents = (rawExtents - shiftExtents)*zoomScale + shiftExtents; 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Animation (Real Time)                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

if ~saveAnimation
    iphase = 1;
    tic
    timeNow = toc*timeRate;
    while iphase <= length(Data)
        if timeNow > Data(iphase).time(end)
            iphase = iphase + 1;
        else
            plotFrame(Data(iphase),timeNow,P);
            timeNow = toc*timeRate;
        end
    end
    
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                       Animation (Render to File)                        %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
else
    if P.Animation.save
        Tspan = [Data(1).time(1), Data(end).time(end)];
        nFrames = diff(Tspan)*frameRate;
        time = linspace(Tspan(1),Tspan(2),nFrames);
        iphase = 1;
        MovieData = zeros(nFrames,0);
        for i=1:nFrames
            if time(i) > Data(iphase).sol.x(end)
                iphase = iphase + 1;
            end
            plotFrame(Data(iphase),time(i),P);
            MovieData(i) = getframe(gcf);
        end
        
        videoObject = VideoWriter('savedAnimation','MPEG-4');
        videoObject.FrameRate = P.Animation.frameRate;
        videoObject.open();
        writeVideo(videoObject,MovieData);
        videoObject.close();
    end
    
end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~  SUB FUNCTIONS  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function plotFrame(Data,t,P)

%This function is designed to be used inside of an animation function.
%
% Data is the simulation data for a single phase of the motion
% t = the desired time for plotting
% axisData is a struct of things for formatting the axis
%


% Plotting parameters
PtMass_MarkerSize = 30;
Leg_LineWidth = 3;
FixedCst_LineWidth = 3;
FixedCst_MarkerSize = 25;
Color_1 = 'r';
Color_2 = 'b';
Color_Hip = 'm';

Time = Data.time;
Foot1 = Data.position.footOne;
Foot2 = Data.position.footTwo;
HipAll = Data.position.hip;

%Intepolate to get the right position
F1 = interp1(Time',Foot1',t','cubic');
F2 = interp1(Time',Foot2',t','cubic');
Hip = interp1(Time',HipAll',t','cubic');

clf; hold on;

plot(F1(1),F1(2),[Color_1 '.'],'MarkerSize',PtMass_MarkerSize*P.dynamics.m1^(1/3));
plot(F2(1),F2(2),[Color_2 '.'],'MarkerSize',PtMass_MarkerSize*P.dynamics.m2^(1/3));
plot(Hip(1),Hip(2),[Color_Hip '.'],'MarkerSize',PtMass_MarkerSize*P.dynamics.M^(1/3));

plot([F1(1), Hip(1)],[F1(2), Hip(2)],[Color_1 '-'],'LineWidth',Leg_LineWidth);
plot([F2(1), Hip(1)],[F2(2), Hip(2)],[Color_2 '-'],'LineWidth',Leg_LineWidth);

switch Data.phase
    case 'D'
        phaseName = 'Double Stance';
        plot(F1(1),F1(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
        plot(F2(1),F2(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
    case 'S1'
        phaseName = 'Single Stance One';
        plot(F1(1),F1(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
    case 'S2'
        phaseName = 'Single Stance Two';
        plot(F2(1),F2(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
    case 'F'
        phaseName = 'Flight';
    otherwise
        error('Invalid phase!')
end

title(['Simulation Time: ' sprintf('%4.2f',t) ', Phase: ' phaseName]);
axis(P.animation.extents); axis equal; axis manual;
xlabel('Horizontal Position (m)')
ylabel('Vertical Position (m)')

drawnow;


end


function extents = getExtents(Data)

%This function is just gets the extents of all of the data in the animation
%by keeping track of min and max values in all phases.

xMin = inf;
xMax = -inf;
yMin = inf;
yMax = -inf;

for iphase = 1:length(Data)
    
    xDat = [Data(iphase).position.footOne(1,:),...
        Data(iphase).position.footTwo(1,:),...
        Data(iphase).position.hip(1,:)];
    
    yDat = [Data(iphase).position.footOne(2,:),...
        Data(iphase).position.footTwo(2,:),...
        Data(iphase).position.hip(2,:)];
    
    xMinTest = min(xDat);
    if xMinTest < xMin
        xMin = xMinTest;
    end
    
    xMaxTest = max(xDat);
    if xMaxTest > xMin
        xMax = xMaxTest;
    end
    
    yMinTest = min(yDat);
    if yMinTest < yMin
        yMin = yMinTest;
    end
    
    yMaxTest = max(yDat);
    if yMaxTest > yMax
        yMax = yMaxTest;
    end
  
end

extents = [xMin, xMax, yMin, yMax];

end

