function [value, isterminal, direction] = groundEvent(~,z)
%
% This function is called by ode45 event-detection to determine when a
% collision with the ground occurs.
%

height = z(2,:);  %Vertical position of the cannon ball

value = height;  
isterminal = true;
direction = -1;

end