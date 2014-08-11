function setup = selectPhase(setup)

%Initialize conditions
valid.hinge = false;
valid.slidePos = false;
valid.slideNeg = false;
valid.flight = false;
P = setup.P;

%Check hinge
IC = [setup.IC.th;
    setup.IC.dth];
eventFunc = @(t,z)events_hinge(t,z,P);
val = feval(eventFunc,setup.Tspan(1),IC);
if sum(val<0)==0 && sum(isnan(val))==0
    if  setup.IC.y == 0 && ...
            setup.IC.dx == 0 && ...
            setup.IC.dy ==0
        valid.hinge = true;
    end
end

%Check flight
IC = [  setup.IC.th;
    setup.IC.x;
    setup.IC.y;
    setup.IC.dth;
    setup.IC.dx;
    setup.IC.dy];
eventFunc = @(t,z)events_flight(t,z,P);
val = feval(eventFunc,setup.Tspan(1),IC);
if sum(val<0)==0 && sum(isnan(val))==0
    valid.flight = true;
end

%Check sliding positive
IC = [  setup.IC.th;
    setup.IC.x;
    setup.IC.dth;
    setup.IC.dx];
eventFunc = @(t,z)events_slidePos(t,z,P);

val = feval(eventFunc,setup.Tspan(1),IC);
if sum(val<0)==0 && sum(isnan(val))==0
    if  setup.IC.y == 0 && ...
            setup.IC.dy ==0
        dZ = dynamics_slidePos([],IC,P);
        if dZ(4) > 0
            valid.slidePos = true;
        end
    end
end

%Check sliding negative
IC = [  setup.IC.th;
    setup.IC.x;
    setup.IC.dth;
    setup.IC.dx];
eventFunc = @(t,z)events_slideNeg(t,z,P);

val = feval(eventFunc,setup.Tspan(1),IC);
if sum(val<0)==0 && sum(isnan(val))==0
    if  setup.IC.y == 0 && ...
            setup.IC.dy ==0
        dZ = dynamics_slideNeg([],IC,P);
        if dZ(4) < 0
            valid.slideNeg = true;
        end
    end
end

%Now pick the right case:
if valid.hinge
    setup.phase = 'HINGE';
elseif valid.slidePos
    setup.phase = 'SLIDE_POS';
elseif valid.slideNeg
    setup.phase = 'SLIDE_NEG';
elseif valid.flight
    setup.phase = 'FLIGHT';
else
    error('No valid phase found');
end

end

