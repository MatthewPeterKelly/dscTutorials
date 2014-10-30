function nLinkAnimate(sol,P)

%This function is used to run an animation of the double pendulum:

duration = sol.x(end) - sol.x(1);

tic;  %Start a timer
timeNow = 0;

h = gcf;

len = sum(P.l);
axisVec = [-len,len,-len,len];

s = sqrt(P.m);
s = s/min(s);

c = hsv(length(P.m)); 

while timeNow < duration
    
    zNow = deval(sol,timeNow);
    
    figure(h);
    plotFrame(timeNow,zNow,P,s,c);
    axis(axisVec); axis square; axis off;
    drawnow;
    
    pause(0.001);
    timeNow = toc;
end

end

function plotFrame(time,z,P,s,c)

N = length(P.m);

[px,py,gx,gy] = eval(['position_' num2str(N) '_link(z,P);']);

Px = [0;px];
Py = [0;py];

clf;
hold on;

%Plot base
plot(0,0,'ks','MarkerSize',20,'LineWidth',3);
plot(0,0,'k.','MarkerSize',25);

%Plot links
for i=1:N
    plot([Px(i) Px(i+1)], [Py(i) Py(i+1)],'-','color',c(i,:),'LineWidth',4);
end

%Plot joints:
for i=1:(N)
    plot(px(i),py(i),'k.','MarkerSize',15);
end

%Plot center of mass:
for i=1:N
plot(gx(i),gy(i),'ko','MarkerSize',round(10*s(i)),'LineWidth', 2);
plot(gx(i),gy(i),'kx','MarkerSize',round(10*s(i)),'LineWidth', 2);
end

title(sprintf('Time: %4.3f',time));

end