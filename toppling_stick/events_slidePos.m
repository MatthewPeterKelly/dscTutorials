function  [value, isterminal, direction] = events_slidePos(~,Z,P)

[~, contacts] = dynamics_slidePos([],Z,P);
th = Z(1,:);
dx = Z(4,:);
H = contacts(1,:);
V = contacts(2,:);

%%%% CONSTRAINTS %%%%
% 1) -pi/2 < th
% 2) th < pi/2
% 3) dx >= 0
% 4) V >= 0
% 5) H <= 0

% As a rule, each constraint will be satisfied if it's event function value
% is positive. This makes things easier at the FSM level.
n = length(th);
value = zeros(5,n);
isterminal = true(size(value));
direction = -ones(size(value));

%%%% HACK %%%%
% avoid the singularity in the dynamics at th = pi/2
th_crit = pi/2 - 1e-10;  
%%%% DONE %%%%

value(1,:) = th + th_crit ;
value(2,:) = th_crit - th;
value(3,:) = dx;
value(4,:) = V;
value(5,:) = -H;

end