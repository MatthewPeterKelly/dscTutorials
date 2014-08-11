function  [value, isterminal, direction] = events_flight(~,Z,P)

LL = P.FullLength;
th = Z(1,:);
y0 = Z(3,:);
y2 = y0 + LL*cos(th);

%%%% CONSTRAINTS %%%%
% 1) y0 > 0
% 2) y2 > 0

% As a rule, each constraint will be satisfied if it's event function value
% is positive. This makes things easier at the FSM level.
n = length(th);
value = zeros(2,n);
isterminal = true(size(value));
direction = -ones(size(value));

value(1,:) = y0;
value(2,:) = y2;

end