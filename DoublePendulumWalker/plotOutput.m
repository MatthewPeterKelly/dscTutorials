function plotOutput(output)

FigNum = 1;

sol = output.result.solution;

idx = output.result.setup.auxdata.index;

Time = sol.phase.time;
th = sol.phase.state(:,idx.THETA);
phi = sol.phase.state(:,idx.PHI);
Dth = sol.phase.state(:,idx.DTHETA);
Dphi = sol.phase.state(:,idx.DPHI);

uHip = sol.phase.control(:,idx.HIP);
uAnk = sol.phase.control(:,idx.ANK);

%Print things:
disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
disp(['Push-off impulse: ' num2str(sol.parameter) ' Nm']);
disp(['Objective value: ' num2str(output.result.objective)...
    ' (' output.result.setup.auxdata.cost.model ')']);
disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
figure(FigNum); clf;

TitleFontSize = 16;
LabelFontSize = 14;
AxisFontSize = 10;

    subplot(3,2,1)
    plot(Time,th,'LineWidth',3)
    title('Stance Angle','FontSize',TitleFontSize)
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Angle (rad)','FontSize',LabelFontSize)
    set(gca,'fontsize',AxisFontSize);
    
    subplot(3,2,2)
    plot(Time,phi,'LineWidth',3)
    title('Swing Angle','FontSize',TitleFontSize)  
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Angle (rad)','FontSize',LabelFontSize)
    set(gca,'fontsize',AxisFontSize);

    subplot(3,2,3)
    plot(Time,Dth,'LineWidth',3)
    title('Stance Rate','FontSize',TitleFontSize)
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Rate (rad/s)','FontSize',LabelFontSize)
    set(gca,'fontsize',AxisFontSize);

    subplot(3,2,4)
    plot(Time,Dphi,'LineWidth',3)
    title('Swing Rate','FontSize',TitleFontSize)  
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Rate (rad/s)','FontSize',LabelFontSize)
    set(gca,'fontsize',AxisFontSize);

    subplot(3,2,5)
    plot(Time,uHip,'LineWidth',3)
    title('Hip Torque','FontSize',TitleFontSize)
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Torque (Nm)','FontSize',LabelFontSize)
    set(gca,'fontsize',AxisFontSize);
    
    subplot(3,2,6)
    plot(Time,uAnk,'LineWidth',3)
    title('Ankle Torque','FontSize',TitleFontSize)
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Torque (Nm)','FontSize',LabelFontSize)
    set(gca,'fontsize',AxisFontSize);
    
end
