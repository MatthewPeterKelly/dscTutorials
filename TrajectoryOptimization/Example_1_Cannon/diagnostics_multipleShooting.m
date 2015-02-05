function diagnostics_multipleShooting(target,param)
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
    
    %%% Print the convergence details:
    if exist('hText','var'), delete(hText); end;
    xText = plotAxis(1) + 0.05*diff(plotAxis(1:2));
    yText = plotAxis(3) + 0.9*diff(plotAxis(3:4));
    textString = {...
        sprintf('1st-order optimal: %3.3e',optimVal.firstorderopt);
        sprintf('constraint violation: %3.3e',optimVal.constrviolation)};
    hText = text(xText,yText,textString,'FontSize',14,'FontWeight','light');
    
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


% %
% %
% %
% %
% %
% %
% %
% %
% %
% %
% %
% % function stop = outFun(decVar,optimVal,state,target,param)
% % % This function is only used for visualization. It has nothing to do with
% % % the single shooting method.
% %
% % global ITER_LOG_MULTIPLESHOOTING;  %Keeping track of iteration details
% %
% % stop = false;
% %
% % switch state
% %     case 'init'
% %         figure(102); clf; hold on;
% %         ITER_LOG_MULTIPLESHOOTING.fVal = [];
% %         ITER_LOG_MULTIPLESHOOTING.iter = 0;
% %         maxHeight = 0.5*target.x + target.y;
% %         ITER_LOG_MULTIPLESHOOTING.axis = ...  %Force the axis to be consistent
% %             [-0.2*target.x, 1.2*target.x,...  %horizontal limits
% %             -0.2*maxHeight, 1.2*maxHeight]; %Vertical limits
% %
% %         plotTraj_multipleShooting([], target, ITER_LOG_MULTIPLESHOOTING.iter);
% %         axis(ITER_LOG_MULTIPLESHOOTING.axis);
% %         drawnow; pause(param.diagnostics.plotPause);
% %         if param.diagnostics.writeGif
% %             set(gcf,'Position',[100,100,param.diagnostics.gifPixelDim]);
% %             frame = getframe(gcf);
% %             im = frame2im(frame);
% %             [imind,cm] = rgb2ind(im,256);
% %             imwrite(imind,cm,param.diagnostics.gifName,'gif',...
% %                 'Loopcount',inf,...
% %                 'DelayTime',param.diagnostics.plotPause);
% %         end
% %     case 'iter'
% %         nState = 4;  %Number of states in the problem (x,y,dx,dy)
% %         nSegment = param.nSegment;
% %         tEnd = decVar(1);
% %         z0 = reshape(decVar(2:end),nState,param.nSegment);
% %
% %         % Run a simulation from the start of each segment, in parallel
% %         nSub = param.nSubStep;  %Number of sub-steps for the integration method
% %         tSim = linspace(0,tEnd/nSegment, nSub+1);
% %         z = rk4_cannon(tSim,z0,param.c); %Simulate the trajectory
% %
% %         %%% Store details about each iteration:
% %         ITER_LOG_MULTIPLESHOOTING.fVal = [ITER_LOG_MULTIPLESHOOTING.fVal; optimVal.fval];
% %         ITER_LOG_MULTIPLESHOOTING.iter = ITER_LOG_MULTIPLESHOOTING.iter + 1;
% %
% %         plotTraj_multipleShooting(z, target, ITER_LOG_MULTIPLESHOOTING.iter);
% %         axis(ITER_LOG_MULTIPLESHOOTING.axis);
% %         drawnow; pause(param.diagnostics.plotPause);
% %
% %         if param.diagnostics.writeGif
% %             frame = getframe(gcf);
% %             im = frame2im(frame);
% %             [imind,cm] = rgb2ind(im,256);
% %             imwrite(imind,cm,param.diagnostics.gifName,'gif','WriteMode','append');
% %         end
% %     case 'done'
% %         hold off
% %     otherwise
% % end
% %
% % end
% %
% %
% % %%%% Special plotting function for use inside of FMINCON
% %
% % function plotTraj_multipleShooting(z, target, iter)
% % % Plots a cannon ball trajectory (x,y), with speed trajectory (dx,dy)
% % % target.x = horizontal target
% % % target.y = vertical target
% % %
% %
% % drawTraj = iter > 0;
% %
% % % Plot the solution:
% % hold on;
% % if ~drawTraj  %Then draw the background
% %     xGround = [-0.2*target.x, 1.2*target.x];
% %     yGround = [0, 0];
% %     plot(xGround, yGround,'color',[0.5, 0.2, 0.1],'LineWidth',4);
% %     drawTree(0.6*target.x, 0, 1);  %Plot a tree for scale
% %     plot(0,0,'b.','MarkerSize',35);   %Start
% %     plot(target.x,target.y,'rx','LineWidth',4,'MarkerSize',14)   %Finish
% % else  %Draw the trajectory
% %     nSegment = size(z,2);
% %     for i=1:nSegment
% %         x = z(1,i,:); x = x(:);
% %         y = z(2,i,:); y = y(:);
% %         plot(x,y,'k-','LineWidth',3);  %Trajectory
% %     end
% % end
% %
% % title(sprintf('Cannon Ball Trajectory:   (iteration = %2d)',iter));
% %
% % axis equal; hold off;
% % xlabel('Horizontal Position')
% % ylabel('Vertical Position')
% %
% % end