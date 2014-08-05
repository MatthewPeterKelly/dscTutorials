%PLOTTING SCRIPT

%Each iteration makes length(x) function calls, where each function call is
%a perturbation experiment. The large-scale solution will only change once
%per iteration, so we count iterations instead of function calls for
%plotting.

iterationNum = floor(CALLNUM/length(x));
showIteration = P.disp.showIterationNum;
ShowPlot = sum(showIteration==iterationNum);  %Plot this solution?

%Make the decision whether or not to plot:
if (ShowPlot && mod(CALLNUM,length(x))==0);   %Only plot the first time!
    
   %set up the parameters for plotting style 
        FigNum = 100000 + iterationNum;
        DefectLineWidth = P.disp.DefectLineWidth;
        DefectLineType = 'r--';

        TitleFontSize = P.disp.TitleFontSize;
        LabelFontSize = P.disp.LabelFontSize;
        AxisFontSize = P.disp.AxisFontSize;
        CurveLineWidth = P.disp.LineWidth;
        
   %Format data for plotting:
        Torque = X.torque;
        Time = linspace(0,X.duration,P.nGridPts);

        Start_Pts = X.state;
        End_Pts = [yPlus,yUpp_RK4];

        Defect_X = [Start_Pts(1,:)', End_Pts(1,:)'];
        Defect_Y = [Start_Pts(2,:)', End_Pts(2,:)']; 

        Traj_X = [yLow(1,:)', yUpp_RK4(1,:)'];
        Traj_Y = [yLow(2,:)', yUpp_RK4(2,:)'];

%Create an empty figure:        
figure(FigNum); clf; hold on;

    %Plot the state-space trajectory
        subplot(3,2,[1,3]); hold on;
        set(gca,'fontsize',AxisFontSize);
        for i=1:size(Defect_X,1)
        plot(Defect_X(i,:),Defect_Y(i,:),DefectLineType,'LineWidth',DefectLineWidth)
        end
        for i=1:size(Traj_X,1)
        plot(Traj_X(i,:),Traj_Y(i,:),'k-','LineWidth',CurveLineWidth)
        end
        xlabel('Angle (rad)','FontSize',LabelFontSize)
        ylabel('Rate (rad/s)','FontSize',LabelFontSize)
        title(['Iteration: ' num2str(iterationNum)],'FontSize',TitleFontSize);

    %Plot the current cost function:
        CostCurve = [0, cumsum(Cost_Segments)];
        subplot(3,2,5); hold on;
        set(gca,'fontsize',AxisFontSize);
        plot(Time,CostCurve,'k-','LineWidth',CurveLineWidth);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Cost','FontSize',LabelFontSize)
        
    %Plot the time-based trajectory
        for i=1:size(Defect_X,1)  %Plot the Defects
        subplot(3,2,2);hold on;
        plot(Time(i)*[1,1], Defect_Y(i,:),DefectLineType,'LineWidth',DefectLineWidth)
        subplot(3,2,4);hold on;
        plot(Time(i)*[1,1],Defect_X(i,:),DefectLineType,'LineWidth',DefectLineWidth)
        end
        for i=1:size(Traj_X,1)   %Plot the sections of trajectories
        subplot(3,2,2);hold on;
        plot([Time(i),Time(i+1)],Traj_Y(i,:),'k-','LineWidth',CurveLineWidth)
        subplot(3,2,4);hold on;
        plot([Time(i),Time(i+1)],Traj_X(i,:),'k-','LineWidth',CurveLineWidth)
        end
        subplot(3,2,6);hold on;   %Plot the Torque
        plot(Time,Torque,'k-','LineWidth',CurveLineWidth)

    %Add some annotations to the axis
        subplot(3,2,2);hold on;
        set(gca,'fontsize',AxisFontSize);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Angle (rad)','FontSize',LabelFontSize)
        title(['Total Cost: ' num2str(Total_Cost)],'FontSize',TitleFontSize); 
        subplot(3,2,4);hold on;
        set(gca,'fontsize',AxisFontSize);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Rate (rad/s)','FontSize',LabelFontSize)
        subplot(3,2,6);hold on;
        set(gca,'fontsize',AxisFontSize);
        xlabel('Time (s)','FontSize',LabelFontSize)
        ylabel('Torque (Nm)','FontSize',LabelFontSize)

    %Now, save the figure if desired:
        Save_Figure_Script

end