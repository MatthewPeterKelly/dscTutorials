function [stateOut, ballRolling] = impactMap(stateIn,P)
%
% This function computes the impact map for the collision that occurs when
% the ball hits the ground.
%
% It is assumed that the collision occurs instantaneously, and that the
% coefficient of restitution is applied normal to the ground at the
% point of impact. The tangential component of the velocity is
% unaffected by the collision. 
% 
% INPUTS:
%   stateIn = the state immediately before the collision
%   P = parameter struct, defined in Set_Paramters.m
%
% OUTPUTS:
%   stateOut = the state immediately after the collision
%   ballRolling = did the ball start rolling?
%

posOut = stateIn(1:2,:);   %The position is not affected by the collision

velIn = stateIn(3:4,:);   %The velocity before the collision

horizPos = stateIn(1,:);  %The horizontal position of the collision

%Get the slope of the ground at the collision location
[~,groundSlope] = groundHeight(horizPos);   

%Rename the coefficient of restitution
e = P.coeff_restitution; 

%Get the angle between the horizontal axis and the tangent to the ground
theta = atan2(groundSlope,1);

%Get the rotation matrix that rotates the velIn vector to be in the
%coordinate frame that is orientated with the slope of the ground
R = [...
    cos(theta)  sin(theta);
    -sin(theta) cos(theta)];

%Get the collision map for a simple collision, that occurs when the axis
%are aligned nicely (only the vertical component is affected)
E = [1 0;
     0 -e];

% Compute the collision
%   This matrix calculation does the following, in order:
%       1) rotate the input velocity
%       2) compute the impact map in rotated coordinates
%       3) rotate the output velocity back to the original coordinates

%Normal and Tangential velocity components
velNT = E*(R*velIn);

%Rotate the velocity back to the inertial coordinates
velOut = R\velNT;

%Pack up the position and velocity and then return
stateOut = [posOut;velOut];

%Check if the ball should start rolling:
if velNT(1)/velNT(2) > P.rollingThreshold
   %The ball will start rolling, because its velocity is nearly tangent
   %to the collision surface:
   disp('The ball is assumed to start rolling, due to the small normal velocity after the collision.');
    ballRolling = true;
else
    ballRolling = false;
end
    

end