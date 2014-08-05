function plotFrame(SimDat,t,P)

%This function is designed to be used inside of an animation function.
%
% SimDat is the simulation data for a single phase of the motion
% t = the desired time for plotting
% axisData is a struct of things for formatting the axis
%

persistent focalPoint;   %Store the center of the camera image
persistent switchTime;   %Store the last time that the phase switched to F
persistent switchPoint;  %The position of the point at the last switch
persistent lastPhase;    %Store the last phase

if nargin==0   %Reset the persistent variables
   focalPoint = [];
   switchTime = [];
   switchPoint = [];
   lastPhase = [];
else

    %Make sure that time is in bounds to avoid error in deval
    if (t>SimDat.sol.x(end))
        State = SimDat.sol.y(:,end);
    elseif (t<SimDat.sol.x(1))
         State = SimDat.sol.y(:,1);
    else
        State = deval(SimDat.sol,t);
    end

    S = convert(State');
    Positions.footOne = [S.x1, S.y1]; 
    Positions.footTwo = [S.x2, S.y2]; 
    Positions.hip = [S.x0, S.y0]; 

    Positions.CoM = ...
        (P.Dyn.m1 * Positions.footOne +... 
        P.Dyn.m2 * Positions.footTwo +...
        P.Dyn.M * Positions.hip)/...
        (P.Dyn.m1 + P.Dyn.m2 + P.Dyn.M);
    
    %Get size of the viewing window
    combinedLegLength = P.Control.legOne.nomPos + P.Control.legTwo.nomPos;
    zoom = P.Animation.zoomScale;
    extents = zoom*combinedLegLength*[-1,1,-1,1];

    %Camera Tracking Algorithm:
    %   If just switched, linearly move camera to the interest point
    %   -> Interest Point = 
    %       S1 = Foot 1
    %       S2 = Foot 2
    %       D = mean(Foot 1 + Foot 2)
    %       F = CoM
    switch SimDat.phase
        case 'D'
            interestPoint = 0.5*(Positions.footOne + Positions.footTwo);
        case 'F'
            interestPoint = Positions.CoM;
        case 'S1'
            interestPoint = Positions.footOne;
        case 'S2'
            interestPoint = Positions.footTwo;
    end
    
    %Reset tracking each time the phase changes
    if ~strcmp(lastPhase,SimDat.phase)
        lastPhase = SimDat.phase;
        switchTime = t;
        switchPoint = focalPoint;
    end
        
    if isempty(focalPoint)   %Then first function call of simulation
        focalPoint = Positions.CoM;
        switchPoint = Positions.CoM;
        switchTime = 0;
        lastPhase = SimDat.phase;
    else  %Normal function call
        trackingDuration = P.Animation.tracking*SimDat.duration;
        tTrans = (t-switchTime);
        if tTrans<trackingDuration  %move to interestPoint
            alpha = tTrans/trackingDuration;
            focalPoint = (1-alpha)*switchPoint + alpha*interestPoint;
        elseif strcmp(SimDat.phase,'F') %Track interestPoint if moving
            focalPoint = interestPoint;
        end
    end    

    %Make the viewing window track the center of mass
    extents(1:2) = extents(1:2) + focalPoint(1);
    extents(3:4) = extents(3:4) + focalPoint(2); 

    F1 = Positions.footOne;
    F2 = Positions.footTwo;
    Hip = Positions.hip;

    clf; hold on;

    PtMass_MarkerSize = 30;  
    plot(F1(1),F1(2),'r.','MarkerSize',PtMass_MarkerSize*P.Dyn.m1^(1/3));
    plot(F2(1),F2(2),'b.','MarkerSize',PtMass_MarkerSize*P.Dyn.m2^(1/3));
    plot(Hip(1),Hip(2),'m.','MarkerSize',PtMass_MarkerSize*P.Dyn.M^(1/3));

    Leg_LineWidth = 3;
    plot([F1(1), Hip(1)],[F1(2), Hip(2)],'r-','LineWidth',Leg_LineWidth);
    plot([F2(1), Hip(1)],[F2(2), Hip(2)],'b-','LineWidth',Leg_LineWidth);
    
    FixedCst_LineWidth = 3;
    FixedCst_MarkerSize = 25;
    switch SimDat.phase
        case 'D'
            phaseName = 'Double Stance';
            plot(F1(1),F1(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
            plot(F2(1),F2(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
        case 'S1'
            phaseName = 'Single Stance One';
            plot(F1(1),F1(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
        case 'S2'
            phaseName = 'Single Stance Two';
            plot(F2(1),F2(2),'ko','MarkerSize',FixedCst_MarkerSize,'LineWidth',FixedCst_LineWidth);
        case 'F'
            phaseName = 'Flight';
        otherwise
            error('Invalid phase!')    
    end

    title(['Simulation Time: ' sprintf('%4.2f',t) ', Phase: ' phaseName]);
    axis(extents); axis equal; axis manual;
    xlabel('Horizontal Position (m)')
    ylabel('Vertical Position (m)')

    drawnow;

end
end