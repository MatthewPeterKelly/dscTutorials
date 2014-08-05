function X = Initial_State(P)

if ~P.MS.Initialize_from_File

    X.duration = mean(P.Bnd.duration);

    Ns = P.MS.nGrid - 2;   %We already know the initial and final states!

    %Linear interpolation for the state initialization. 
    X.state = zeros(4,Ns);
    for i=1:4
       tmp = linspace(P.MS.Start(i),P.MS.Finish(i),Ns+2);
       X.state(i,:) = tmp(2:(end-1));
    end

    %Vector for the actuator force at each grid point
    X.force = zeros(1,P.MS.nGrid);
    
    
else  %Then we are going to initialize from a file:
    
    uiopen('*.mat');  
    %We will assume that the data file has the variable Results with a
    %field Xsoln, that contains the previous solution.
    
    N = length(Results.Xsoln.force);
    if N == P.MS.nGrid   %Then same grid spacing
        X = Results.Xsoln;
    else
        X.duration = Results.Xsoln.duration;
        
        domain = [0,Results.Xsoln.duration];
        gridOld = linspace(domain(1),domain(2),Results.P.MS.nGrid);
        gridNew = linspace(domain(1),domain(2),P.MS.nGrid);
        
        statesOld = [Results.P.MS.Start, Results.Xsoln.state, Results.P.MS.Finish];
        statesNew = interp1(gridOld',statesOld',gridNew')';
        X.state = statesNew(:,2:(end-1));
        
        X.force = interp1(gridOld',Results.Xsoln.force',gridNew')';
    end
    
end