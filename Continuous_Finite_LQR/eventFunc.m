function [value, isterminal, direction] = eventFunc(~,z)
%This function is used to stop the simulation if it goes outside of the
%domain of the system.

value = z.^2 - 1;  
isterminal = true(size(z));
direction = zeros(size(z));
end