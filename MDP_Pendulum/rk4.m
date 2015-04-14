function z = rk4(dyn,t,z0)
% z = rk4(dyn,t,z0)
%
% This function (rk4) is used to perform a 4th-order Runge-Kutta
% integration of a dynamical system.
%
% INPUTS:
%   dyn = handle of the form:
%       dz = userFunc(t,z)
%   t = [1 x nTime] vector of times, created by linspace
%   z0 = [nState (x nSim)] matrix of initial states
%
% OUTPUTS:
%   z = [nState (x nSim) x nTime ] matrix of trajectories
%

[nState,nSim] = size(z0);
nTime = length(t);

if nSim==1
    z = zeros(nState,nTime);
    z(:,1) = z0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = dyn(t(i),  z(:,i));
        k2 = dyn(t(i)+0.5*dt,  z(:,i) + 0.5*dt*k1);
        k3 = dyn(t(i)+0.5*dt,  z(:,i) + 0.5*dt*k2);
        k4 = dyn(t(i)+dt,  z(:,i) + dt*k3);
        z(:,i+1) = z(:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end
else
    z = zeros(nState,nSim,nTime);
    z(:,:,1) = z0;
    for i=1:(nTime-1)
        dt = t(i+1)-t(i);
        k1 = dyn(t(i),  z(:,:,i));
        k2 = dyn(t(i)+0.5*dt,  z(:,:,i) + 0.5*dt*k1);
        k3 = dyn(t(i)+0.5*dt,  z(:,:,i) + 0.5*dt*k2);
        k4 = dyn(t(i)+dt,  z(:,:,i) + dt*k3);
        z(:,:,i+1) = z(:,:,i) + (dt/6)*(k1+2*k2+2*k3+k4);
    end    
end

end