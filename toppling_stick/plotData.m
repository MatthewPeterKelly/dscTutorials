function plotData(D)

data = D.data;
Jumps = D.Jumps;
P = D.P;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           States                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(1); clf;
set(figH,'Name','States','NumberTitle','off')

subplot(4,1,1);hold on;
plot(data.time,data.state.th);
ylabel('Angle (rad)')
xlabel('Time (s)')
dottedLine(Jumps,axis);

subplot(4,1,2); hold on;
plot(data.time,data.state.dth);
ylabel('Rate (rad)')
xlabel('Time (s)')
dottedLine(Jumps,axis);

subplot(4,1,3); hold on
plot(data.time,data.state.x,'r');
plot(data.time,data.state.y,'b');
legend('x','y');
ylabel('position (m)')
xlabel('Time (s)')
dottedLine(Jumps,axis);

subplot(4,1,4); hold on
plot(data.time,data.state.dx,'r');
plot(data.time,data.state.dy,'b');
legend('dx','dy');
ylabel('velocity (m/s)')
xlabel('Time (s)')
dottedLine(Jumps,axis);




%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                                Traces                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

L = P.L;
LL = P.FullLength;
x0 = data.state.x;
y0 = data.state.y;
x1 = x0 - L*sin(data.state.th);
y1 = y0 + L*cos(data.state.th);
x2 = x0 - LL*sin(data.state.th);
y2 = y0 + LL*cos(data.state.th);
bnd = [min(min(x0),min(x2)),max(max(x0),max(x2))];

figH = figure(3); clf; hold on;
set(figH,'Name','Traces','NumberTitle','off')

plot(bnd,[0,0],'k-','LineWidth',1);
plot(x0,y0,'r','LineWidth',3)
plot(x1,y1,'m','LineWidth',3)
plot(x2,y2,'b','LineWidth',3)
legend('ground','O','CoM','Tip');
axis 'equal';
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Forces and Energy                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
figH = figure(2); clf;
set(figH,'Name','Energy','NumberTitle','off')

subplot(3,1,1); hold on;
plot(data.time,data.contact.h,'r');
plot(data.time,data.contact.v,'b');
legend('Horizontal','Vertical')
ylabel('Contact Force (N)')
xlabel('Time (s)')
dottedLine(Jumps,axis);

subplot(3,1,2); hold on;
E_pot = data.energy.potential;
E_kin = data.energy.kinetic;
plot(data.time,E_pot,'b');
plot(data.time,E_kin,'r');
plot(data.time,E_kin + E_pot,'k','LineWidth',2);
legend('Potential','Kinetic','Total','Location','NorthWest')
ylabel('Energy (J)')
xlabel('Time (s)')
dottedLine(Jumps,axis);

subplot(3,1,3); hold on;
plot(data.time,E_kin + E_pot,'k','LineWidth',2);
ylabel('Total Energy (J)')
xlabel('Time (s)')
dottedLine(Jumps,axis);

figH = figure(4); clf; hold on
set(figH,'Name','Contacts','NumberTitle','off')

th = data.state.th*180/pi;
plot(th,data.contact.h,'r');
plot(th,data.contact.v,'b');
legend('Horizontal','Vertical')
ylabel('Contact Force (N)')
xlabel('Angle (deg)')

end

%%%% SUB FUNCTIONS %%%%

function dottedLine(time,AXIS)

for i=1:length(time)
    %Plots a dotted line between phases
    plot(time(i)*[1;1],[AXIS(3);AXIS(4)],'k:','LineWidth',1);
end

end