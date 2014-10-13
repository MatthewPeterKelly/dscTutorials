function doublePendulumAnimate(sol,l1,l2)

%This function is used to run an animation of the double pendulum:

duration = sol.x(end) - sol.x(1);

tic;  %Start a timer
timeNow = 0;

h = gcf;

len = l1+l2;
axisVec = [-len,len,-len,len];

while timeNow < duration
    
    zNow = deval(sol,timeNow,[1,3]);
    th1 = zNow(1);
    th2 = zNow(2);
    [p1,p2] = doublePendulumPosition(th1,th2,l1,l2);
    
    figure(h);
    plotFrame(timeNow,p1,p2)
    axis(axisVec); axis square;
    drawnow;
    
    pause(0.001);
    timeNow = toc;
end

end

function plotFrame(time,p1,p2)

linkOneX = [0;p1(1)];
linkOneY = [0;p1(2)];

linkTwoX = [p1(1);p2(1)];
linkTwoY = [p1(2);p2(2)];

clf;
hold on;

plot(linkOneX,linkOneY,'r-','LineWidth',8);
plot(linkTwoX,linkTwoY,'b-','LineWidth',8);

title(sprintf('Time: %4.3f',time));

end