function plotPendulumCart(t,z)
% plotPendulumCart(t,z)
%
% INPUTS:
%   t = [1, n] = time stamps for each state vector
%   z = [4, n] = [x;q;dx;dq] = state of the system
%
% OUTPUTS:
%   a simple plot for each state of the system over the simulation
%

%%%% Unpack the state:
x = z(1,:);
q = z(2,:);
dx = z(3,:);
dq = z(4,:);

%%%% Plots:

subplot(2,2,1);
plot(t,x)
ylabel('x')
title('Position')

subplot(2,2,2);
plot(t,q)
ylabel('q')
title('Angle')

subplot(2,2,3);
plot(t,dx)
ylabel('dx')
title('Velocity')

subplot(2,2,4);
plot(t,dq)
ylabel('dq')
title('Angle Rate')

end