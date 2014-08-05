function Animate(timeAll,stateAll,stateOde,P)
%
% This function is used to animate the simulation data
%
% INPUTS:
%   timeAll = [1xN] vector of monotonically increasing time stamps
%   stateAll = [4xN] matrix of state vectors at each time in timeAll
%   stateOde = [4xM] matrix of state vectors returned by ode45
%
% OUTPUTS:
%   A plot and animation displayed on the current figure
%

clf;

%break out the data:
    xPos = stateAll(1,:);
    yPos = stateAll(2,:);
    Pos = [xPos;yPos];

%Get the ground shape
    xGround = linspace(min(xPos),max(xPos),1000);   
    yGround = groundHeight(xGround);

%Get the extents for plotting:
    Extents = [min(xPos),max(xPos),min(yGround),max(yPos)];

    
tic; %Start a timer for synchronizing the animation   

endTime = timeAll(end);
SlowMo = P.slowMotionFactor;
cpuTime=toc/SlowMo;
counter = 1;  %Keeps track of the current plotting index
while (cpuTime<endTime)  
    
    cpuTime = toc/SlowMo;
    while (timeAll(counter)<cpuTime) && counter<length(timeAll)
        %Advance the counter to catch up with real time
        counter=counter+1;
    end
    PosNow = Pos(:,counter);  %Get the current position, based on counter
    
    %Check if the ball has left the screen:
    if PosNow(2) < Extents(3)
       %if the ball height is less than the lower part of the plot
       disp('The ball left the field of view!')
       break;
    end
    
    %clear the figure
    clf; hold on; axis(Extents); axis equal; axis manual;
    
    %Plot the ball
    plot(PosNow(1),PosNow(2),'b.','MarkerSize',P.PlotBallSize); 
    
    %Plot the trace
    plot(Pos(1,1:counter),Pos(2,1:counter),'b--','LineWidth',max(P.CurveLineWidth-1,1)); 
    
    %Plot the ground
    plot(xGround,yGround,'k-','LineWidth',P.CurveLineWidth);

    %Annotations
    ylabel('Vertical Position (m)','FontSize',P.LabelFontSize)
    xlabel('Horizontal Position (m)','FontSize',P.LabelFontSize)
    title('Path taken by the ball','fontsize',P.TitleFontSize)
    set(gca,'fontsize',P.AxisFontSize);    
    
    %Force the plot to appear:
    drawnow; pause(0.001);
    
end

%Plot the points that ode45 returned
plot(stateOde(1,:),stateOde(2,:),'.r','MarkerSize',ceil(P.PlotBallSize/3));

ylabel('Vertical Position (m)','FontSize',P.LabelFontSize)
xlabel('Horizontal Position (m)','FontSize',P.LabelFontSize)
title('Path taken by the ball','fontsize',P.TitleFontSize)
hLegend = legend('ball','trajectory','ground','ode45 gridpoints');
set(gca,'fontsize',P.AxisFontSize);
set(hLegend,'fontsize',P.LegendFontSize);

end