function [value,isterminal,direction] = EventFunction(~,X)

x = X(1,:);
y = X(2,:);
ground = groundHeight(x);

value = y-ground;

%Stop if the hit the ground
isterminal = true;

%Should only be coming to the ground from above
direction = -1;

end