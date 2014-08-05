function [Xlow, Xupp] = Set_Bounds(P)

Ns = P.MS.nGrid - 2;   %We already know the initial and final states!
LOW = 1;
UPP = 2;

%% Lower Bounds:
Xlow.duration = P.Bnd.duration(LOW);
Xlow.state = P.Bnd.state(:,LOW)*ones(1,Ns);
Xlow.force = P.Bnd.force(LOW)*ones(1,P.MS.nGrid);

%% Upper Bounds:
Xupp.duration = P.Bnd.duration(UPP);
Xupp.state = P.Bnd.state(:,UPP)*ones(1,Ns);
Xupp.force = P.Bnd.force(UPP)*ones(1,P.MS.nGrid);

end