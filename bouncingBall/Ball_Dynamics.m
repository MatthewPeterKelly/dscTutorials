function dZ = Ball_Dynamics(~,Z,P)

%This function computes the dynamics of the ball as it travels through the
%air. The ball is treated as a point mass, acting under the uniform
%acceleration of gravity and quadratic drag from the air.
%
% INPUTS:
%   (~) = time (not used)
%   Z = a [4xN] matrix of states, where each column is a state vector
%   P = a parameter struct created by Set_Parameters.m
%
% OUTPUTS:
%   dZ = a [4xN] matrix giving the derivative of Z with respect to time
%

%Unpack the physical parameters
    g = P.gravity;  %(m/s^2) gravitational acceleration
    m = P.mass;     %(kg) mass of the ball
    c = P.drag;     %(N*s^2/m^2) quadratic drag coefficient
    
%Get the velocity vector
    % pos = Z(1:2,:);   %Position vector is not used
    vel = Z(3:4,:);   %Velocity vector
    
% Compute gravity and drag accelerations
    speed = sqrt(vel(1,:).^2+vel(2,:).^2);      %(m/s) scalar speed
    Drag = -c*vel.*speed/m;  %(m/s^2) quadratic drag
    Gravity = [0;-g]*ones(size(speed));       %(m/s^2) gravitational acceleration
    
% Return derivative of the state
    acc = Drag + Gravity;
    dZ = [vel;acc];
    
end