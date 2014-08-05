function PlotStates(timeAll, stateAll, P)
    
clf;

    %break out the data:
        time = timeAll;
        xPos = stateAll(1,:);
        yPos = stateAll(2,:);
        xVel = stateAll(3,:);
        yVel = stateAll(4,:);


    subplot(2,2,1)
        plot(time,xPos,'LineWidth',P.CurveLineWidth);
        xlabel('time (s)','FontSize',P.LabelFontSize)
        ylabel('horizontal position (m)','FontSize',P.LabelFontSize)
        set(gca,'fontsize',P.AxisFontSize);
    subplot(2,2,2)
        plot(time,yPos,'LineWidth',P.CurveLineWidth);
        xlabel('time (s)','FontSize',P.LabelFontSize)
        ylabel('vertical position (m)','FontSize',P.LabelFontSize)
        set(gca,'fontsize',P.AxisFontSize);
    subplot(2,2,3)
        plot(time,xVel,'LineWidth',P.CurveLineWidth);
        xlabel('time (s)','FontSize',P.LabelFontSize)
        ylabel('horizontal speed (m/s)','FontSize',P.LabelFontSize)
        set(gca,'fontsize',P.AxisFontSize);
    subplot(2,2,4)
        plot(time,yVel,'LineWidth',P.CurveLineWidth);
        xlabel('time (s)','FontSize',P.LabelFontSize)
        ylabel('vertical speed (m/s)','FontSize',P.LabelFontSize)
        set(gca,'fontsize',P.AxisFontSize);

end