function Plot_Double_Pendulum(tout,xout,E,P)

%FUNCTION:
%   This function is used to generate plots showing how the states and
%   system energy vary with time. 
%
%INPUTS:
%   tout = a vector of time, (tout, returned by ode45)
%   xout = a matrix of states corresponding to tout, also returned by ode45
%   E = Energetics struct, returned by Double_Pendulum_Kinematics.m
%   P = Parameters struct, returned by Set_Parameters.m
%
%OUTPUTS:
%   Figure 2  --  States vs Time
%   Figure 3  --  Energy vs Time

%Set which figures to plot on
figNumStates = 2;
figNumEnergy = 3;

%Get the font and line sizes
    TitleFontSize = P.plot.TitleFontSize;
    LabelFontSize = P.plot.LabelFontSize;
    AxisFontSize = P.plot.AxisFontSize;
    CurveLineWidth = P.plot.CurveLineWidth;

    %Plot States
    figure(figNumStates); clf; hold on;
        Theta = xout(:,1)';
        Phi = xout(:,2)';
        dTheta = xout(:,3)';
        dPhi = xout(:,4)';
        Time = tout';
    subplot(2,2,1)
        plot(Time,Theta,'LineWidth',CurveLineWidth);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Abs Angle 1 (rad)','FontSize',LabelFontSize)
        set(gca,'fontsize',AxisFontSize);
    subplot(2,2,2)
        plot(Time,Phi,'LineWidth',CurveLineWidth);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Abs Angle 2 (rad)','FontSize',LabelFontSize)
        set(gca,'fontsize',AxisFontSize);
    subplot(2,2,3)
        plot(Time,dTheta,'LineWidth',CurveLineWidth);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Abs Rate 1 (rad/s)','FontSize',LabelFontSize)
        set(gca,'fontsize',AxisFontSize);
    subplot(2,2,4)
        plot(Time,dPhi,'LineWidth',CurveLineWidth);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Abs Rate 2 (rad/s)','FontSize',LabelFontSize)
        set(gca,'fontsize',AxisFontSize);

    %Plot Energy
    figure(figNumEnergy); clf; hold on;
    plot(tout,E.Potential,'r-','LineWidth',CurveLineWidth)
    plot(tout, E.Kinetic, 'b-','LineWidth',CurveLineWidth)
    plot(tout,E.Total, 'k-','LineWidth',CurveLineWidth)
            ylabel('Energy (J)','FontSize',LabelFontSize)
            xlabel('Time (s)','FontSize',LabelFontSize)
    title('Mechanical Energy','FontSize',TitleFontSize)
    legend('Potential','Kinetic','Total')
    set(gca,'fontsize',AxisFontSize);

end


