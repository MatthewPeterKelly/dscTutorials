function star = getStarVerticies(n,r)
% star = getStarVerticies(n,r)
%
% This function draws a star with n points centered on the origin.
%
% INPUTS:
%   n = number of points
%   r = inner radius (outer radius == 1)
%
% OUTPUTS:
%   star = [2, 2*n+1] = [x;y] coordinates of verticies
%

th = linspace(0,2*pi,2*n+1);
star = [...
    sin(th);
    cos(th)];
idx = 2:2:(2*n);
star(:,idx) = r*star(:,idx);

end