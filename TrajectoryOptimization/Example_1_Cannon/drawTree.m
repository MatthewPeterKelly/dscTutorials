function drawTree(x,y,h)
% function drawTree(x,y,h)
% 
% This function is used to draw a pine tree, with trunk at position (x,y),
% with the desired height (h). It is used for giving a scale to a figure.
%

xGreen = x + h*[0.0, 0.2, 0.1, 0.3, 0.16, 0.4, -0.4,  -0.16, -0.3, -0.1, -0.2, 0.0];
yGreen = y + h*[1.0, 0.75, 0.75, 0.50, 0.50, 0.2, 0.2, 0.50, 0.50, 0.75, 0.75, 1.0];

xBrown = x + h*0.08*[1,1,-1,-1];
yBrown = y + h*0.2*[0,1,1,0];

patch(xGreen, yGreen, [0.1,0.6,0.1]);
patch(xBrown, yBrown, [0.4, 0.2, 0.1]);

end