clc;

%This code runs a simulation of a single spring pendulum, running all
%dynamics and integration in C++ code. Matlab is only used for plotting and
%compiling the code (to a mex function).

%First, compile the most recent version of the file
Recompile_Code = true;
if Recompile_Code
    mex Cpp_Integrator.cpp;    
else
    disp('Warning - Did not recompile C++ files!')
    disp(' ')
end

% Initial state for the system
% [x,y,dx,dy]';
x0 = [  0;
        -0.4;   
        0.6;
        0];   %Must be a column vector
Tspan = [0,20];
N = 1e5;


Params = zeros(1,6);  %Must be a row vector!
Params(1) = 0.8;    %(kg)   Mass
Params(2) = 89;    %(N/m)  Spring Constant
Params(3) = 0;    %(Ns/m)  Damping
Params(4) = 9.81;   %(m/s^2)  Gravity
Params(5) = 0.2;    %(m) Rest length of spring
Params(6) = 0.5;    %(Nm)    Forcing Amplitude
Params(7) = 20;      %(Hz)   Forcing frequency (Hz)



tic
[t, X] = Cpp_Integrator(Tspan,x0,N,Params);
disp(' ')
disp(['C++ Time: (Normal) ' num2str(toc)]);

x = X(1,:);
y = X(2,:);
dx = X(3,:);
dy = X(4,:);

figure(1)
plot(t,X)
title('All States')

figure(2)
plot(x,y)
title('Position Trace')
