function Critical = ToppleFromRest(P)

%This function uses conservation of energy to compute the dynamics as the
%stick topples from rest. This assumes that toppling will be in the
%positive theta direction.

if P.u==0
    Critical.th = eps;
    Critical.exit = 'SlipBackwards';
    Critical.dth = eps;
    Critical.H = 0;
else
    %%%% Find the critical angle at which slipping occurs:
    SMALL_ANGLE = 1e-4;
    userFunc = @(t,z)rhs(t,z,P);
    Tspan = [0,10];
    options = odeset(...
        'RelTol',1e-12,...
        'AbsTol',1e-12,...
        'Events',@(t,z)EventFunc(t,z,P));
    sol = ode45(userFunc,Tspan,SMALL_ANGLE,options);
    
    if isempty(sol.ie)
        Critical.exit = 'TimeOut';
    else
        switch sol.ie
            case 1
                Critical.exit = 'Fall';
            case 2
                Critical.exit = 'SlipForwards';
            case 3
                Critical.exit = 'SlipBackwards';
        end
    end
    Critical.th = sol.y(end);
    Critical.dth = topple_angularRate(Critical.th,P);
    Critical.H = topple_contactHorizontal(Critical.th,P);
end
end

function dth = rhs(~,th,P)

dth = topple_angularRate(th,P);

end


function [value, isterminal, direction] = EventFunc(~,th,P)

u = P.u;

%All constraints are satisfied if the value is positive, all events are
%checked only when decreasing from positive to negative
N = length(th);
value = zeros(3,N);
isterminal = true(3,N);
direction = -ones(3,N);

V = topple_contactVertical(th,P);
H = topple_contactHorizontal(th,P);

value(1,:) = pi/2 - th;
if ~isinf(u)
    value(2,:) = u*V - H;
    value(3,:) = H + u*V;
else
    value(2,:) = V;  %Always slips forwards with infinite friction
    value(3,:) = 0;
end

end
