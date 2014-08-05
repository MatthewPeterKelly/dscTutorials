function AngleMod = WrapAngle(Angle)

%This function takes an angle and maps it to the range [-pi,pi];

Angle = Angle + pi;   %Shift up

    MinAng = min(Angle);
    while MinAng < 0
        Angle = Angle + 2*pi;  %Wrap around again
        MinAng = min(Angle);
    end

    AngleMod = mod(Angle,2*pi);
    
AngleMod = AngleMod - pi;     %Shift back

end