function [value, isterminal, direction] = eventFunc(~,z,xLim,yLim)
%This function is used to stop the simulation if it goes outside of the
%domain of the system.

value = [...
    z(1,:) - xLim(1);
    z(1,:) - xLim(2);
    z(2,:) - yLim(1);
    z(2,:) - yLim(2)];
isterminal = true(size(value));
direction = zeros(size(value));
end