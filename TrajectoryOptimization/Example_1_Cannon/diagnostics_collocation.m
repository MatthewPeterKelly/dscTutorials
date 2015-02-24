function diagnostics_collocation(target,param,soln)
% This function is used for generating various plots and statistics about
% the optimization run for the cannon problem.
%

global ITER_LOG_COLLOCATION; %Contains information about each iteration

figure(param.diagnostics.figNum.collocation); clf;
gifName = 'cannon_directCollocation.gif';

nIter = length(ITER_LOG_COLLOCATION);

plotAxis = plotBackground(target,param); hold on;
color = rainbow(nIter);  %Color map for trajectories

%%% Stuff for saving as a GIF
frameRate = param.diagnostics.animationDuration/(nIter+1);
if param.diagnostics.writeGif
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,gifName,'gif',...
        'Loopcount',inf,...
        'DelayTime',frameRate);
end

%%% Loop over each iteration and plot the trajectory
for iter=1:nIter
    optimVal = ITER_LOG_COLLOCATION(iter).optimVal;
    decVar = ITER_LOG_COLLOCATION(iter).decVar;
    tEnd = decVar(1);  %Trajectory duration
    
    nState = 4;  %Number of states in the problem (x,y,dx,dy)
    nSegment = param.collocation.nSegment;
    nGrid = nSegment + 1; %Number of sub-steps for the integration method
    z = reshape(decVar(2:end),nState,nGrid);
    
    %%% Hermite-Simpson Quadrature is implicitly integrating a cubic
    %%% interpolating polynomial - we will just use Matlab to do this:
    t = linspace(0,tEnd,nGrid);
    f = cannonDynamics(t,z,param.dynamics.c);
    PC = pwch(t,z,f);  %Piecewise-Cubic Interpolating polynomial
    
    % Draw the trajectory
    tPlot = linspace(0,tEnd,5*nSegment+1);
    zPlot = ppval(PC,tPlot);
    x = zPlot(1,:);
    y = zPlot(2,:);
    plot(x,y,'k-','LineWidth',3,'color',color(iter,:));  %Trajectory
    
    title(sprintf('Collocation Method:   (iteration = %2d)',iter));
    axis(plotAxis); drawnow;
    pause(frameRate);
    
    if ~soln.success
        textColor = 'r';
    else
        textColor = 'k';
    end
    
    %%% Print the convergence details:
    if exist('hText','var'), delete(hText); end;
    xText = plotAxis(1) + 0.05*diff(plotAxis(1:2));
    yText = plotAxis(3) + 0.9*diff(plotAxis(3:4));
    textString = {...
        sprintf('1st-order optimal: %3.3e',optimVal.firstorderopt);
        sprintf('constraint violation: %3.3e',optimVal.constrviolation);
        sprintf('initial speed: %3.3f',sqrt(soln.cost))};
    hText = text(xText,yText,textString,'FontSize',14,...
        'FontWeight','light','Color',textColor);
    
    %%% Stuff for saving as a GIF
    if param.diagnostics.writeGif
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        imwrite(imind,cm,gifName,'gif',...
            'WriteMode','append',...
            'DelayTime',frameRate);
    end
    
end

end
