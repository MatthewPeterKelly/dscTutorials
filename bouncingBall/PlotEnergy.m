function PlotEnergy(timeAll, stateAll, P)

clf; hold on;

%break out the data:
    time = timeAll;
    %xPos = stateAll(1,:);
    yPos = stateAll(2,:);
    xVel = stateAll(3,:);
    yVel = stateAll(4,:);

    m = P.mass;
    g = P.gravity;
    
%Kinetic energy  
    KE = 0.5*m*(xVel.^2+yVel.^2);

%Potential energy
    PE =  m*g*yPos;

%Total energy
    Energy = KE+PE;

%Plot the system energy
    plot(timeAll,KE,'b-','LineWidth',P.CurveLineWidth)
    plot(timeAll,PE,'r-','LineWidth',P.CurveLineWidth)
    plot(timeAll,Energy,'k-','LineWidth',P.CurveLineWidth+2)
    hLegend = legend('kinetic','potential','total');
    title('System Energy','fontsize',P.TitleFontSize)
    xlabel('Time (s)','FontSize',P.LabelFontSize)
    ylabel('Energy (J)','FontSize',P.LabelFontSize)
    set(gca,'fontsize',P.AxisFontSize);
    set(hLegend,'fontsize',P.LegendFontSize);

end