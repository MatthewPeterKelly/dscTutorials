function diagnostics_singleShooting(target,param,soln)
% This function is used for generating various plots and statistics about
% the optimization run for the cannon problem.
%

global ITER_LOG_SINGLESHOOTING; %Contains information about each iteration

figure(param.diagnostics.figNum.singleShooting); clf;
gifName = 'cannon_singleShooting.gif';

nIter = length(ITER_LOG_SINGLESHOOTING);
nGrid = param.singleShooting.nGrid;

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

    if ~soln.success
        textColor = 'r';
    else
        textColor = 'k';
    end  
    

%%% Loop over each iteration and plot the trajectory
for iter=1:nIter
    optimVal = ITER_LOG_SINGLESHOOTING(iter).optimVal;
    decVar = ITER_LOG_SINGLESHOOTING(iter).decVar;
    T = decVar(3);  %Trajectory duration
    time = linspace(0,T,nGrid);  %Build grid in time
    z0 = [0;0;decVar(1); decVar(2)];  %[x; y; dx; dy];
    z = rk4_cannon(time,z0,param.dynamics.c); %Simulate the trajectory
    x = z(1,:);  y=z(2,:);
    
    %%% Plotting here:
    plot(x,y,'color',color(iter,:),'LineWidth',3);  %Trajectory
    title(sprintf('Single Shooting Method:   (iteration = %2d)',iter));
    axis(plotAxis); drawnow;
    pause(frameRate);

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