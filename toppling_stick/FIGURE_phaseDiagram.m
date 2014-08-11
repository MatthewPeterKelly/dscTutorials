%FIGURE_Phase_Diagram

clear;

MoI = logspace(-3,0,100);
Mu = logspace(-2,2,100);
[xx,yy] = meshgrid(MoI,Mu);
zz = zeros(size(xx));
type = zeros(size(xx));
P.m = 1;
P.g = 1;
P.L = 1;

for i=1:size(xx,1)
    disp(['Mu= ' num2str(yy(i,1))]);
    for j=1:size(yy,2)
        P.I = xx(i,j);
        P.u = yy(i,j);
        Critical = ToppleFromRest(P);
            zz(i,j) = Critical.th;
            switch Critical.exit
                case 'Fall'
                    type(i,j) = 0;
                case 'SlipForwards'
                    type(i,j) = 1;
                case 'SlipBackwards'
                    type(i,j) = -1;
                otherwise
                    type(i,j) = 0;
            end
    end
end

figure(300); clf; hold on;
N = length(MoI)*length(Mu);
x = reshape(xx,N,1);
y = reshape(yy,N,1);
z = reshape(zz,N,1);
t = reshape(type,N,1);
plot3(x(t==1),y(t==1),z(t==1),'r.','MarkerSize',20);
plot3(x(t==-1),y(t==-1),z(t==-1),'b.','MarkerSize',20);
plot3(x(t==0),y(t==0),z(t==0),'k.','MarkerSize',20);
set(gca,'Xscale','log');
set(gca,'Yscale','log');
xlabel('MoI');
ylabel('Mu');
zlabel('Critical Angle');

figure(301); clf; 
contour(xx,yy,type,[-0.5,0.5])
set(gca,'Xscale','log');
set(gca,'Yscale','log');
xlabel('MoI');
ylabel('Mu');
zlabel('Critical Angle');

figH = figure(302); clf; hold on;
set(figH,'Name','SlipModeRaw','NumberTitle','off')
FontSize.title = 24;
FontSize.label = 16;
idx = t==0; plot(x(idx),y(idx),'k.','MarkerSize',4)
idx = t==-1; plot(x(idx),y(idx),'kx','MarkerSize',4)
idx = t==1; plot(x(idx),y(idx),'ko','MarkerSize',4)
set(gca,'Xscale','log');
set(gca,'Yscale','log');
set(gca,'FontSize',14);

xlabel('Moment of Inertia','FontSize',FontSize.label)
ylabel('Coefficient of Friction','FontSize',FontSize.label)
title('Slip Mode' ,'FontSize',FontSize.title)

text(1e-2,1e1,{'  SLIP  ', 'FORWARDS'},'FontSize',30)
text(1e-2,1e-1,{'  SLIP  ', 'BACKWARDS'},'FontSize',30)
text(0.34,11,{' NO ', 'SLIP'},'FontSize',30)

%%%% THE GOOD FIGURE: %%%%

figH = figure(303); clf; hold on;
set(figH,'Name','SlipMode','NumberTitle','off')
FontSize.title = 24;
FontSize.label = 16;
Bnd = getBoundaries(xx,yy,type);
plot(Bnd(:,1),Bnd(:,2),'k.','MarkerSize',25);
set(gca,'Xscale','log');
set(gca,'Yscale','log');
set(gca,'FontSize',14);

xlabel('Moment of Inertia','FontSize',FontSize.label)
ylabel('Coefficient of Friction','FontSize',FontSize.label)
title('Slip Mode' ,'FontSize',FontSize.title)

text(1e-2,1e1,{'  SLIP  ', 'FORWARDS'},'FontSize',30)
text(1e-2,1e-1,{'  SLIP  ', 'BACKWARDS'},'FontSize',30)
text(0.34,11,{' NO ', 'SLIP'},'FontSize',30)


%%%% SAVE %%%%
save('DATA_SlipMode.mat');
save2pdf('../WriteUp/Figures/SlipMode.pdf',figH,600);




