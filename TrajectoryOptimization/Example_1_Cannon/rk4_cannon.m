function z = rk4_cannon(t,z0,P)
% z = rk4_cannon(dyn,t,z0,P)
%
% This function is used to perform a 4th-order Runge-Kutta
% integration of the cannon dynamical system
%
% INPUTS:
%   t = [1 x nTime] vector of times, created by linspace
%   z0 = [nState (x nSim)] matrix of initial states
%   P = parameter to pass to dynamics
%
% OUTPUTS:
%   z = [nState (x nSim) x nTime ] matrix of trajectories
%
% NOTES:
%   1) See rk4.m for the generic version of this function
%   2) I've hard-coded the dynamics function into this version because it
%      is faster than calling the anonymous function with an embedded
%      parameter.
%

[nState,nSim] = size(z0);
nTime = length(t);

if nSim==1
    z = zeros(nState,nTime);
    z(:,1) = z0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = cannonDynamics(t(i),  z(:,i),P);
        k2 = cannonDynamics(t(i)+0.5*dt,  z(:,i) + 0.5*dt*k1,P);
        k3 = cannonDynamics(t(i)+0.5*dt,  z(:,i) + 0.5*dt*k2,P);
        k4 = cannonDynamics(t(i)+dt,  z(:,i) + dt*k3,P);
        z(:,i+1) = z(:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end
else
    z = zeros(nState,nSim,nTime);
    z(:,:,1) = z0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = cannonDynamics(t(i),  z(:,:,i),P);
        k2 = cannonDynamics(t(i)+0.5*dt,  z(:,:,i) + 0.5*dt*k1,P);
        k3 = cannonDynamics(t(i)+0.5*dt,  z(:,:,i) + 0.5*dt*k2,P);
        k4 = cannonDynamics(t(i)+dt,  z(:,:,i) + dt*k3,P);
        z(:,:,i+1) = z(:,:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end    
end

end