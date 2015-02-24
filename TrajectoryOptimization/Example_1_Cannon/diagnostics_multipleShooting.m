function diagnostics_multipleShooting(target,param,soln)
% This function is used for generating various plots and statistics about
% the optimization run for the cannon problem.
%

global ITER_LOG_MULTIPLESHOOTING; %Contains information about each iteration

figure(param.diagnostics.figNum.multipleShooting); clf;
gifName = 'cannon_multipleShooting.gif';

nIter = length(ITER_LOG_MULTIPLESHOOTING);

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
    optimVal = ITER_LOG_MULTIPLESHOOTING(iter).optimVal;
    decVar = ITER_LOG_MULTIPLESHOOTING(iter).decVar;
    tEnd = decVar(1);  %Trajectory duration
    
    nState = 4;  %Number of states in the problem (x,y,dx,dy)
    nSegment = param.multipleShooting.nSegment;
    nSubStep = param.multipleShooting.nSubStep; %Number of sub-steps for the integration method
    z0 = reshape(decVar(2:end),nState,nSegment);
    
    % Run a simulation from the start of each segment, in parallel
    tSim = linspace(0,tEnd/nSegment, nSubStep+1);
    z = rk4_cannon(tSim,z0,param.dynamics.c); %Simulate the trajectory
    
    for i=1:nSegment
        x = z(1,i,:); x = x(:);
        y = z(2,i,:); y = y(:);
        plot(x,y,'k-','LineWidth',3,'color',color(iter,:));  %Trajectory
    end
    title(sprintf('Multiple Shooting Method:   (iteration = %2d)',iter));
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
        'FontWeight','light','color', textColor);
    
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

