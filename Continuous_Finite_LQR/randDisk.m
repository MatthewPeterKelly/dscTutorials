function samples = randDisk(nSamples,radius,center)
% randDisk(nSamples,radius,center)
%
% Compute a random distribution of points inside a unit disk.
%
% INPUTS:
%   nSamples = number of random samples to return
%   radius = optional radius parameter. Default = 1.
%       radius = R   --> circle with radius R
%       radius = [a;b;th]  --> ellipse
%           a = major-axis radius
%           b = minor-axis radius
%           th = angle between major-axis and x-axis
%       center = [cx, cy] optional center of the circle command
%
% OUTPUTS:
%   samples = [x;y] a uniform distribution within the desired region
%
%

r = sqrt(rand(1,nSamples));
phi = 2*pi*rand(1,nSamples);
[x,y] = pol2cart(phi,r);

if nargin==1   %Simple unit disk
    samples = [x;y];
else %Then we have some scaling to do:
    if length(radius)==1  %The scale the circle uniformly
        x = radius*x;
        y = radius*y;
        samples = [x;y];
    elseif length(radius)==2  %The plain ellipse
        a = radius(1); b = radius(2);
        x = a*x;
        y = b*y;
        samples = [x;y];
    elseif length(radius)==3  %Then ellipse with rotation!
        a = radius(1); b = radius(2); th = -radius(3);
        x = a*x;
        y = b*y;
        R = [cos(th), sin(th); -sin(th), cos(th)];
        S = [x;y];
        samples = (R*S);
    else
        error('Invalid input for radius');
    end
    
    if nargin>2 %Then shift the whole thing
        samples(1,:) = samples(1,:) + center(1);
        samples(2,:) = samples(2,:) + center(2);
    end
end

end