% reachableForwardRel
%
% A script for computing the forward reachable set, using an relative
% coordinate system. It seems to give roughly reasonable reasonable
% results, but there seems to be some numerical dissapation during the run.
% For example, points that we know are stable appear to be unstable, which
% I think is due to the feature size of the reachable set approaching the
% grid-spacing. This will occur for any grid spacing, since it seems like
% the reachable set becomes an very long and thin ellipsoid-like shape.
%

nReach = 40;  %Number of times at which to display level sets
if (tSpan(2)~=0), error('tSpan(2) must be equal to 0 for reachability calculations!'); end;
% tReach = linspace(tSpan(1),tSpan(2),nReach);
 tReach = linspace(tSpan(1),-1.5,nReach);

radius = 0.3;

nGrid = 400;  %Discritization of the state space in each dimension
grid.dim = 2;  %Two-dimensional problem
grid.min = 8*radius*[-1;-1];
grid.max = 8*radius*[1;1];
grid.dx = (1/(nGrid-1))*(grid.max-grid.min);
grid.bdry = @addGhostExtrapolate;
grid = processGrid(grid);

% Create initial conditions (a circle around the target)
center = [0;0];   %We're working in relative coordinates here!
data = shapeSphere(grid, center, radius); %Initialize level-set function
levelSet = 0;


% Set up the level-set problem:
schemeFunc = @termConvection;
schemeData.velocity = @reachableDynamicsRel;
schemeData.grid = grid;

schemeData.fit.x = xFit;
schemeData.fit.y = yFit;
schemeData.fit.u = uFit;
schemeData.fit.kx = kxFit;
schemeData.fit.ky = kyFit;

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.5, 'stats', 'on');

% Choose approximations at appropriate level of accuracy.
accuracy = 'medium';
switch(accuracy)
    case 'low'
        schemeData.derivFunc = @upwindFirstFirst;
        integratorFunc = @odeCFL1;
    case 'medium'
        schemeData.derivFunc = @upwindFirstENO2;
        integratorFunc = @odeCFL2;
    case 'high'
        schemeData.derivFunc = @upwindFirstENO3;
        integratorFunc = @odeCFL3;
    case 'veryHigh'
        schemeData.derivFunc = @upwindFirstWENO5;
        integratorFunc = @odeCFL3;
    otherwise
        error('Unknown accuracy level %s', accuracy);
end

%Data structure for storing the reachable set, as contour lines:
Reach(nReach).C = [];

%Store the initial contour line:
Reach(1).C = getContour(grid,data,levelSet);

%Run the backwards reachability calculation:
startTime = cputime;
for idx = 1:(nReach-1)
    
    %Display data to user:
    figure(12); clf; contour(grid.xs{1},grid.xs{2}, data);
    
    % Reshape data array into column vector for ode solver call.
    y0 = data(:);
    
    % How far to step?
    tSpanReach = [tReach(idx), tReach(idx+1)];
    
    % Take a timestep.
    [t, y] = feval(integratorFunc, schemeFunc, tSpanReach, y0,...
        integratorOptions, schemeData);
    if t~=tSpanReach(2)
        error('Something funny happened!')
    end
    
    % Get back the correctly shaped data array
    data = reshape(y, grid.shape);
    
    if min(min(data)) > 0, break;   end
    
    %Store the level curve:
    Reach(idx+1).C = getContour(grid,data,levelSet);
    
end

%Plot the results:
figure(7); clf; hold on;
colors = jet(nReach);
xNomReach = polyval(xFit,tReach);
yNomReach = polyval(yFit,tReach);
for i=1:nReach
    C = Reach(i).C;
    if ~isempty(Reach(i).C)
        for j=1:length(C)
            x = C(j).x + xNomReach(i);
            y = C(j).y +yNomReach(i);
            plot(x,y,'LineWidth',3,'color',colors(i,:));
        end
    end
end
plot(xNomReach,yNomReach,'k.','MarkerSize',20);
plot(xSol,ySol,'k')
xlabel('x');
ylabel('y');
zlabel('t');
title('Forward reachable set')


