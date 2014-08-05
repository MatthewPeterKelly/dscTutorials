function Yplus = HammerImpact(Yminus,P)

%This function models a simple point-mass pendulum bouncing off of the
%groun with a known coefficient of restitution.

e = P.dyn.coeffRestitution;  %Coefficient of restitution

%Initialize memory:
Yplus = zeros(size(Yminus));
Yplus(1,:) = Yminus(1,:);     %Th - angle - no change
Yplus(2,:) = -e*Yminus(2,:);  %W - rate - impact map

end