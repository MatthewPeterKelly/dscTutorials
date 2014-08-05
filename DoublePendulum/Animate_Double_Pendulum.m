function Animate_Double_Pendulum(t,K,P)

%FUNCTION:
%   This function is used to generate an animation of the double pendulum,
%   using precomputed values.
%
%INPUTS:
%   t = a vector of time, (tout, returned by ode45)
%   K = Kinematics struct, returned by Double_Pendulum_Kinematics.m
%   P = Parameters struct, returned by Set_Parameters.m
%
%OUTPUTS:
%   Figure 1  --  Animation of double pendulum
%


%Which figure to plopt on?
figure(1);

%Get the various position vectors
P1_x = K.r_P1_O(1,:);
P1_y = K.r_P1_O(2,:);
P2_x = K.r_P2_O(1,:);
P2_y = K.r_P2_O(2,:);

%Get the font sizes
    TitleFontSize = P.plot.TitleFontSize;
    LabelFontSize = P.plot.LabelFontSize;

%Display a few things:
Parameters_to_Display = {...
        ['Slow Motion Factor: ' num2str(P.sim.slow_motion_factor)];...
        '';...
        ['Mass 1:  ' num2str(P.dyn.m1,4) '  kg'];...   
        ['Mass 2:  ' num2str(P.dyn.m2,4) '  kg'];...  
        ['Gravity:  ' num2str(P.dyn.g,4) '  m/s^2'];...
        ['Length:  ' num2str(P.dyn.L,4) '  m'];...
};

tic;    %Start a timer
Loop_Time = 0;    %store how long has the simulation been running
i=2;   %Start at second index (for interpolation purposes)
Max_i = length(t);
T_end = t(end);   %Ending time of one step
L = P.dyn.L;   %Leg length
Bounds = 1.1 * 2*L * [-1,1,-1,1];

while Loop_Time < T_end;  %Loop while the CPU time is less than the end of the simulation's time
    %The next few lines pull the time step that is closest to the real time
    Loop_Time = toc/P.sim.slow_motion_factor;   %Get the current time  (Taking slow motion into accunt if desired)
    while (i<Max_i) && (Loop_Time > (t(i))) %While we are not at the end of the data and the CPU time is ahead of the simulation time
        i=i+1;   %Go to the next time frame
    end
    %Now t(i-1) < Loop_Time < t(i) should be true
    
    %Linear interpolation scheme
        t_m = t(i-1);
        t_p = t(i);

        Pend_X_m = [0;P1_x(i-1); P2_x(i-1)];
        Pend_Y_m = [0;P1_y(i-1); P2_y(i-1)];

        Pend_X_p = [0;P1_x(i); P2_x(i)];
        Pend_Y_p = [0;P1_y(i); P2_y(i)];

        dPend_X = Pend_X_p - Pend_X_m;
        dPend_Y = Pend_Y_p - Pend_Y_m;
    
        dt = t_p - t_m;
        dt_cpu = Loop_Time - t_m;
        m = dt_cpu/dt;
        
        Pend_X = m*dPend_X + Pend_X_p;
        Pend_Y = m*dPend_Y + Pend_Y_p;
        
    clf
    plot(Pend_X,Pend_Y,'k-','LineWidth',6)
    hold on
    plot(Pend_X,Pend_Y,'b.','MarkerSize',50)
    plot(0,0,'ko','MarkerSize',40,'LineWidth',3)
    title(['Simulation Time: ' num2str(t(i)) ' seconds'],'FontSize',TitleFontSize)
    
    text(Bounds(2),mean(Bounds(3:4)),Parameters_to_Display,'FontSize',LabelFontSize);
    
    axis(Bounds); axis equal, axis manual; axis off;
    drawnow
    
end




end