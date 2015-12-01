% TEST - Human readable dynamics
%
% This is a simple script to check that the "human readable" version of the
% dynamics matches the computer generated version.


p.m1 = 1.0;  % (kg) Cart mass
p.m2 = 0.3;  % (kg) pole mass
p.g = 9.81;  % (m/s^2) gravity 
p.l = 0.5;   % (m) pendulum (pole) length 

z = 10*randn(4,1000);   %Random state
u = 10*randn(1,1000);   %Random control

dzSym = cartPoleDynamics(z,u,p);
dzHuman = cartPoleDynamicsHumanReadable(z,u,p);

err = abs(dzSym-dzHuman);
maxErr = max(max(err));
if maxErr > 1e-12
    error('Dynamics do not match!');
end

fprintf('\n\n Max Error: %8.8g \n\n',max(max(err)));
