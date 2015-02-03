function cost = objective(dx,dy)
% Objective function for optimization. Here we set the objective function
% to be proportional to the initial energy of the cannon ball. Note that

cost = dx.*dx + dy.*dy;

end