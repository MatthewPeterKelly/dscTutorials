function map = rainbow(n)
% map = RAINBOW(n)
%
% This function returns a colormap that ranges from red to blue
%
% INPUTS: 
%   n = number of rows in colormap
%
% OUTPUTS:
%   map = [n x 3] color map
%
%   See also HSV2RGB, JET, HSV, HOT, PINK, FLAG, COLORMAP, RGBPLOT.

hue = linspace(0,2/3,n)';
sat = 0.95*ones(n,1);
val= 0.95*ones(n,1);
map = hsv2rgb([hue,sat,val]);

end