function  [value, isterminal, direction] = events_hinge(~,Z,P)

[~, contacts] = dynamics_hinge([],Z,P);
u = P.u;
th = Z(1,:);
H = contacts(1,:);
V = contacts(2,:);


%%%% CONSTRAINTS %%%%
% 1) -pi/2 < th
% 2) th < pi/2
% 3) V > 0
% 4) H > -u*H
% 5) H < u*H

% As a rule, each constraint will be satisfied if it's event function value
% is positive. This makes things easier at the FSM level.
n = length(th);
value = zeros(5,n);
isterminal = true(size(value));
direction = -ones(size(value));

value(1,:) = th + pi/2;
value(2,:) = pi/2 - th;
value(3,:) = V;

value(4,:) = u*V - H;
value(5,:) = H + u*V;

end