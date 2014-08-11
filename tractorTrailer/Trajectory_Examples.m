%Trajectory_Examples.m
%
% This script contains a few sample "roads". SetPts are a series of points
% along the road, to which a polynomial is fit. Therer are also a few
% simulation parameters associated with each example.
%

Params.Sim.Example_Number = Example_Number;
switch Example_Number
    case 1
        Params.Traj.Func = false;
        Params.Traj.SetPts = [...
            0,0;
            10,5;
            20,9;
            40,9;
            60,0;
            80,-10];
        Params.Traj.Order = 4;
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 0.5;
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 60;
        Control_Frequency = 50;   %Hz
        
    case 2
        %This one appears to work reasonably well
        Params.Traj.Func = false;
        Params.Traj.SetPts = [...
            0,0;
            10,5;
            20,9;
            40,9;
            60,0;
            65,-2.5;
            70, -5;
            75,-8
            80,-12];
        Params.Traj.Order = 4;
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 0.5;
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 50;
        Control_Frequency = 50;   %Hz
        
    case 3
        Params.Traj.Func = false;
        Params.Traj.SetPts = [...
            0 0;
            5 0;
            10 1;
            15 3;
            20 2;
            25 0;
            30 -3;
            35 -8;
            40 -13;
            45 -20;
            50 -30;
            55 -40];
        Params.Traj.Order = 4;
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 1.5;
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 125;
        Control_Frequency = 40;   %Hz
        
        
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
    case 4    
        %USED IN PRESENTATION - MAIN RESULT
        
	    Params.Traj.Func = true;
        x = linspace(0,150,Params.Traj.Npts+1)';
        Params.Traj.x = x;
        Params.Traj.y = sin(x/20).*25.*x.^2/(150^2);
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 1.5;  %used for setting lqr gains
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 90;
        Control_Frequency = 50;   %Hz
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        
 
    case 5  
        % Works
        Params.Traj.Func = true;
        x = linspace(0,150,Params.Traj.Npts+1)';
        Params.Traj.x = x;
        Params.Traj.y = sin(x/15).*25;
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 1.5;
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 160;
        Control_Frequency = 10;   %Hz
        
    case 6
      % USED IN PRESENTATION - Appendix 1
      %Long version of 5 (many turns)  -- good demonstration
        Params.Traj.Func = true;
        Params.Traj.Npts = 250; %Override the number of points
        Params.Traj.Max_Rd_Idx = 245;
        x = linspace(0,350,Params.Traj.Npts+1)';
        Params.Traj.x = x;
        Params.Traj.y = sin(x/20+pi/3).*cos(x/100)*25;
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 1.5;  %used for setting lqr gains
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 235;
        Control_Frequency = 50;   %Hz
        
    case 7
        %Cosine Road
        Params.Traj.Npts = 150;
        Freq = 0.35;
        Amp = 25;
        Params.Traj.Func = true;
        x = linspace(0,150,Params.Traj.Npts+1)';
        Params.Traj.x = x;
        Params.Traj.y = Amp*cos((2*pi/Amp)*x*Freq+0.1);
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 1.5;
        Params.Ctl.Minimum_Goal_Distance = 1; %(m) how far to "look ahead"
        Simulation_Duration = 250;
        Control_Frequency = 50;   %Hz
        
     case 8
      %REALLY LONG version of 5 (many turns)  -- good demonstration
        Params.Traj.Func = true;
        Params.Traj.Npts = 1000; %Override the number of points
        Params.Traj.Max_Rd_Idx = 995;
        x = linspace(0,350,Params.Traj.Npts+1)';
        Params.Traj.x = x;
        Params.Traj.y = sin(x/20+pi/3).*cos(x/100)*25;
        Params.Sim.IC_Error = [0;0;0;0];
        Max_Pos_Err = 1.5;  %used for setting lqr gains
        Params.Ctl.Minimum_Goal_Distance = 2; %(m) how far to "look ahead"
        Simulation_Duration = 235*4;
        Control_Frequency = 50;   %Hz
        
    otherwise
        error('Invalid Example Number')
        
end
