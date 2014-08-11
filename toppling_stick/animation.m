function animation(D, timeRate)

%This function runs an animation for the finite state machine

figH = figure(10); clf; hold on;
set(figH,'Name','Animation','NumberTitle','off')

%Format data:
P = D.P;
L = P.L;
LL = P.FullLength;
nPhase = length(D.raw);
for i=1:nPhase
    data = D.raw(i);
    x0 = data.state.x;
    y0 = data.state.y;
    th = data.state.th;
    D.plot(i).x0 = x0;
    D.plot(i).y0 = y0;
    D.plot(i).x1 = x0 - L*sin(th);
    D.plot(i).y1 = y0 + L*cos(th);
    D.plot(i).x2 = x0 - LL*sin(th);
    D.plot(i).y2 = y0 + LL*cos(th);
    D.plot(i).time = D.raw(i).time;
    D.plot(i).phase = D.phase{i};
end

% Figure out how big to make the viewing window:
rawExtents = getExtents(D);
zoomScale = 1.1;  %How much to pad the viewing window (>=1);
xMean = mean(rawExtents(1:2));
yMean = mean(rawExtents(3:4));
shiftExtents = [xMean, xMean, yMean, yMean];
P.extents = (rawExtents - shiftExtents)*zoomScale + shiftExtents;
P.timeRate = timeRate;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Animation (Real Time)                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

iphase = 1;
tic
timeNow = toc*timeRate;
while iphase <= length(D.plot)
    if timeNow > D.plot(iphase).time(end)
        iphase = iphase + 1;
    else
        plotFrame(D.plot(iphase),timeNow,P);
        timeNow = toc*timeRate;
    end
end
plotFrame(D.plot(end),D.plot(end).time(end),P);

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

Time = Data.time;
Point0 = [Data.x0', Data.y0'];
Point1 = [Data.x1', Data.y1'];
Point2 = [Data.x2', Data.y2'];

%Intepolate to get the right position
try
    P0 = interp1(Time,Point0,t,'pchip');
    P1 = interp1(Time,Point1,t,'pchip');
    P2 = interp1(Time,Point2,t,'pchip');
catch ME
    %Assume that there is only one grid point in Time, causing an error
    P0 = Point0;
    P1 = Point1;
    P2 = Point2;
end

%Actual plotting happens here
clf; hold on;
xGnd = P.extents(1:2);
yGnd = zeros(size(xGnd));
plot(xGnd,yGnd,'k-','LineWidth',4);
plot(P0(1),P0(2),'r.','MarkerSize',55);
plot(P1(1),P1(2),'m.','MarkerSize',55);
plot(P2(1),P2(2),'b.','MarkerSize',55);
plot([P0(1), P2(1)],[P0(2), P2(2)],'k-','LineWidth',7);

%Make a pretty title and annotation for pinned constraint
switch Data.phase
    case 'HINGE'
        phaseName = 'Pinned Rotation';
        plot(P0(1),P0(2),'ko','MarkerSize',25,'LineWidth',1);
    case 'SLIDE_POS'
        phaseName = 'Sliding to the right';
    case 'SLIDE_NEG'
        phaseName = 'Sliding to the left';
    case 'FLIGHT'
        phaseName = 'Flight phase';
    otherwise
        error('Invalid phase!')
end
title(['Simulation Time: ' sprintf('%4.2f',t) ', --  Phase: ' phaseName],'FontSize',26);
axis(P.extents); axis equal; axis manual;
xlabel('Horizontal Position (m)','FontSize',20)
ylabel('Vertical Position (m)','FontSize',20)

drawnow;

end


function extents = getExtents(D)

%This function is just gets the extents of all of the data in the animation
%by keeping track of min and max values in all phases.

Data = D.plot;

xMin = inf;
xMax = -inf;
yMin = inf;
yMax = -inf;

for iphase = 1:length(Data)
    
    xDat = [Data(iphase).x0,...
        Data(iphase).x2];
    
    yDat = [Data(iphase).y0,...
        Data(iphase).y2];
    
    xMinTest = min(xDat);
    if xMinTest < xMin
        xMin = xMinTest;
    end
    
    xMaxTest = max(xDat);
    if xMaxTest > xMax
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

