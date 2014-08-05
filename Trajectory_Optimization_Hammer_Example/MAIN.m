%MAIN
% This script is designed to be a demonstration of how to properly set up a
% multiple shooting problem in Matlab. It can also be used to generate some
% intermediate plots for educational purposes.
%
% Hammer-swing trajectory optim$ization; plain english specification:
%
%   Find a periodic trajectory for a simple hammer that is periodically
%   striking a surface. Assume that the hammer is a simple, point mass,
%   pendulum, that is driven by a torque source at it's base. When it
%   strikes the table it bounces back with some known fraction of its
%   original speed. Find the trajectory that minimizes the torque-squared
%   actuator cost while striking the surface at a given speed.

%open _READ_ME.txt
clc; clear; close all;

%% Set up and run trajectory optimization:

%Parameters and bounds:
[P, X] = Get_Parameters();
[Xlow, Xupp] = Get_Bounds(P);

%Function Handles:
objfunc = @(x)Objective_Function(x,P);
nonlcon = @(x)NonLinCon(x,P);

%No linear constraints:
A=[]; Aeq=[]; B=[]; Beq=[];
          
%Store things in a Problem struct:
  Problem.objective = objfunc;
  Problem.x0 = Convert_State(X,P.structDat);
  Problem.Aineq = A;
  Problem.bineq = B;
  Problem.Aeq = Aeq;
  Problem.beq = Beq;
  Problem.lb = Convert_State(Xlow,P.structDat);
  Problem.ub = Convert_State(Xupp,P.structDat);
  Problem.nonlcon = nonlcon;
  Problem.options = P.opt;
  Problem.solver = 'fmincon';
  
%Run fmincon:
tic
  [xSoln, Fval, ExitFlag, Output] = fmincon(Problem);
toc

%Extract useful information from the results
  Xsoln = Convert_State(xSoln,P.structDat);
  Time = linspace(0,Xsoln.duration,P.nGridPts);
  Angle = Xsoln.state(1,:);
  Rate = Xsoln.state(2,:);
  Torque = Xsoln.torque;
  Soln = PhysicsIntegration(xSoln,P);
  
%% Plot things:

  figure(1); clf;
    TitleFontSize = P.disp.TitleFontSize;
    LabelFontSize = P.disp.LabelFontSize;
    AxisFontSize = P.disp.AxisFontSize;
    LineWidth = P.disp.LineWidth;
    
  subplot(3,2,[1,3])  %State Space
    plot(Angle,Rate,'k-','LineWidth',LineWidth); hold on;
    plot(Angle,Rate,'ko','LineWidth',LineWidth,'MarkerSize',10)
    xlabel('Angle (rad)','FontSize',LabelFontSize)
    ylabel('Rate (rad/s)','FontSize',LabelFontSize)
    iterationNum = Output.iterations;  %store the number of iterations
    title(['Iteration: ' num2str(iterationNum)],'FontSize',TitleFontSize);
    
  subplot(3,2,5)  % Cost Function
    CostCurve = [0, cumsum(Soln.Cost_Segments)];
    set(gca,'fontsize',AxisFontSize);
    plot(Time,CostCurve,'k-','LineWidth',LineWidth);
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Cost','FontSize',LabelFontSize)
    
  subplot(3,2,2)  % Angle
    plot(Time,Angle,'k-','LineWidth',LineWidth);
    set(gca,'fontsize',AxisFontSize);
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Angle (rad)','FontSize',LabelFontSize)
    title(['Total Cost: ' num2str(Fval)],'FontSize',TitleFontSize); 
    
  subplot(3,2,4)  % Rate
    plot(Time,Rate,'k-','LineWidth',LineWidth);
    set(gca,'fontsize',AxisFontSize);
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Rate (rad/s)','FontSize',LabelFontSize)
    
  subplot(3,2,6) % Torque
    plot(Time,Torque,'k-','LineWidth',LineWidth);
    set(gca,'fontsize',AxisFontSize);
    xlabel('Time (s)','FontSize',LabelFontSize)
    ylabel('Torque (Nm)','FontSize',LabelFontSize)

%Save the figure (if desired) for use in LaTeX
Save_Figure_Script
