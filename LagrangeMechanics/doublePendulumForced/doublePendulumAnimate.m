function doublePendulumAnimate(sol,P)

%This function is used to run an animation of the double pendulum:

duration = sol.x(end) - sol.x(1);

tic;  %Start a timer
timeNow = 0;

h = gcf;

len = P.l1+P.l2;
axisVec = [-len,len,-len,len];

s1 = sqrt(P.m1);
s2 = sqrt(P.m2);
sMin = min(s1,s2);
s1 = s1/sMin;
s2 = s2/sMin;

while timeNow < duration
    
    zNow = deval(sol,timeNow);
    [p1,p2,g1,g2] = doublePendulumPosition(zNow,P);
    
    figure(h);
    plotFrame(timeNow,p1,p2,g1,g2,s1,s2);
    axis(axisVec); axis square; axis off;
    drawnow;
    
    pause(0.001);
    timeNow = toc;
end

end

function plotFrame(time,p1,p2,g1,g2,s1,s2)

linkOneX = [0;p1(1)];
linkOneY = [0;p1(2)];

linkTwoX = [p1(1);p2(1)];
linkTwoY = [p1(2);p2(2)];

clf;
hold on;


plot(0,0,'ks','MarkerSize',20,'LineWidth',3);

plot(linkOneX,linkOneY,'r-','LineWidth',6);
plot(linkTwoX,linkTwoY,'b-','LineWidth',6);

plot(0,0,'k.','MarkerSize',25);
plot(p1(1),p1(2),'k.','MarkerSize',25);
plot(p2(1),p2(2),'k.','MarkerSize',25);

plot(g1(1),g1(2),'ko','MarkerSize',round(15*s1),'LineWidth', 3);
plot(g2(1),g2(2),'ko','MarkerSize',round(15*s2),'LineWidth', 3);
plot(g1(1),g1(2),'kx','MarkerSize',round(15*s1),'LineWidth', 3);
plot(g2(1),g2(2),'kx','MarkerSize',round(15*s2),'LineWidth', 3);

title(sprintf('Time: %4.3f',time));

end