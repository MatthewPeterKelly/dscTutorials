function [Xlow, Xupp] = Get_Bounds(P)

%Used as enum constants:
LOW = 1;  UPP = 2;  

%Rename the number of gridpoints for convienance:
nGridPts = P.nGridPts;

%Store the low bounds:
  Xlow.duration = P.bnd.duration(LOW);
  Xlow.state = [P.bnd.angle(LOW);
                P.bnd.rate(LOW)]*ones(1,nGridPts);
  Xlow.torque = P.bnd.torque(LOW)*ones(1,nGridPts);
  
%Store the upper bounds:
  Xupp.duration = P.bnd.duration(UPP);
  Xupp.state = [P.bnd.angle(UPP);
                P.bnd.rate(UPP)]*ones(1,nGridPts);
  Xupp.torque = P.bnd.torque(UPP)*ones(1,nGridPts);

%Some of the constraints can be treated as state bounds:
  ImpactState = [P.cst.strikeAngle; P.cst.strikeRate];
  Xlow.state(:,end) = ImpactState;
  Xupp.state(:,end) = ImpactState;
 
  
end