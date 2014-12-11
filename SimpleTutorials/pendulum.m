function pendulum()
% Matlab code for simulating a simple pendulum

P.g = 9.81;     %(m/s^2) gravity acceleration
P.l = 1.0;      %(m) length of pendulum   
P.c = 1.0;     %(1/s) viscous damping constant
P.m = 1.0;     %(kg) pendulum mass

th0 = 0.5;  %(rad) initial angle of pendulum, from -j axis
w0 = 0.0;   %(rad/s) initial angular rate of pendulum
z0 = [th0;w0];   %Initial state vector

tSpan = [0,4]; %(s) [start, end] times for simulation
dtMax = 0.01;  %(s) maximum allowable time step

nStep = ceil(diff(tSpan)/dtMax);  %Number of simulation steps
t = linspace(tSpan(1),tSpan(2),nStep);  %(s) time vector 
z = zeros(2,nStep); %  Initialize the state matrix
z(:,1) = z0;

%Run the simulation, using Euler integration
for i=2:nStep
    dt = t(i)-t(i-1);
    zNow = z(:,i-1);
    z(:,i) = zNow + dt*dynamics(zNow,P);
end

%Generate a plot of the result:
figure(1); clf;
subplot(2,1,1); plot(t,z(1,:));
xlabel('Time (s)'); ylabel('Angle (rad)'); 
title('Simple Damped Pendulum');
subplot(2,1,2); plot(t,z(2,:));
xlabel('Time (s)'); ylabel('Rate (rad/s)'); 

end

function dz = dynamics(z,P)
% Compute the dynamics of a simple damped pendulum:

th = z(1,:); w = z(2,:); %Unpack the state vector

%Dynamics:
dth = w;  
dw = -(P.g/P.l)*sin(th) - (P.c/P.m)*w;

dz = [dth; dw];  %Pack up state derivative
end