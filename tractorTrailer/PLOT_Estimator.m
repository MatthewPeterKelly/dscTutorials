function PLOT_Estimator(Time, Trace_Cov, State_Error)

figure(6);
plot(Time,Trace_Cov)
title('Trace of the covariance matrix')
xlabel('Time (s)')
ylabel('Trace of Covariance')

figure(7)

subplot(4,1,1)
plot(Time, State_Error(1,:))
title('EKF Error: X position', 'FontSize',16)
xlabel('Time (s)')
ylabel('Distance (m)')

subplot(4,1,2)
plot(Time, State_Error(2,:))
title('EKF Error: Y position', 'FontSize',16)
xlabel('Time (s)')
ylabel('Distance (m)')

subplot(4,1,3)
plot(Time, State_Error(3,:))
title('EKF Error: Orientation', 'FontSize',16)
xlabel('Time (s)')
ylabel('Angle (rad)')

subplot(4,1,4)
plot(Time, State_Error(4,:))
title('EKF Error: Cab-Trailer Angle', 'FontSize',16)
xlabel('Time (s)')
ylabel('Angle (rad)')

end