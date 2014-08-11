function PLOT_Inputs(Time,Inputs)

figure(3);
clf;

subplot(2,1,1)
plot(Time,Inputs(1,:))
title('Speed Commands')
xlabel('Time (s)')
ylabel('Speed (m/s)')

subplot(2,1,2)
plot(Time,Inputs(2,:))
title('Steering Commands')
xlabel('Time (s)')
ylabel('Angle (rad)')

end



