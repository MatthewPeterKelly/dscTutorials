%FIGURE_Critical_Angle

MoI = logspace(-3,0,100);
Mu = [0,0.05,0.1,0.5,1,5,10,inf];
P.m = 1;
P.g = 1;
P.L = 1;

Data = cell(length(Mu),0);
for i=1:length(Mu)
    Data(i).idxF = false(size(MoI));
    Data(i).idxB = false(size(MoI));
    Data(i).fail = false(size(MoI));
    Data(i).th = zeros(size(MoI));
    P.u = Mu(i);
    disp(['Running u = ' num2str(P.u)]);
    for j=1:length(MoI)
        P.I = MoI(j);
        C = ToppleFromRest(P);
        if ~isempty(C.th)
            if strcmp(C.exit,'SlipForwards')
               Data(i).idxF(j) = true; 
            elseif strcmp(C.exit,'SlipBackwards')
                Data(i).idxB(j) = true;
            end
            Data(i).th(j) = C.th;
        else
            Data(i).fail(j) = true; 
        end
    end
end

style = {'k--','b--','r--','m--','g--',...
    'k-','b-','r-','m-','g-'...
    'k:','b:','r:','m:','g:'};
LINEWIDTH = 3;
FontSize.Title = 16;
FontSize.label = 12;
names = cell(1,length(Mu));
for i=1:length(Mu)
    names{i} = num2str(['u = ' num2str(Mu(i))]);
end

figH = figure(400); clf;
set(figH,'Name','CriticalAngleLim','NumberTitle','off')
IDX = false(2,length(Mu));
for i=1:length(Mu)
    sty = style{mod(i-1,length(style))+1};
    idx = Data(i).idxB;  IDX(1,i) = sum(idx)~=0;
    subplot(2,1,1);  hold on;
    semilogx(MoI(idx),Data(i).th(idx)*180/pi,sty,'LineWidth',LINEWIDTH);
    idx = Data(i).idxF;  IDX(2,i) = sum(idx)~=0;
    subplot(2,1,2);  hold on;
    semilogx(MoI(idx),Data(i).th(idx)*180/pi,sty,'LineWidth',LINEWIDTH);
end

subplot(2,1,1); 
 title('Backwards Slip (falling forward)','FontSize',FontSize.Title)
    xlabel('Moment of Inertia','FontSize',FontSize.label)
    ylabel('Critical Angle (deg)','FontSize',FontSize.label)
    legend(names(IDX(1,:)),'Location','NorthEast');
    set(gca,'Xscale','log')
    
    subplot(2,1,2); 
 title('Forwards Slip (falling forward)','FontSize',FontSize.Title)
    xlabel('Moment of Inertia','FontSize',FontSize.label)
    ylabel('Critical Angle (deg)','FontSize',FontSize.label)
    legend(names(IDX(2,:)),'Location','NorthWest');
    set(gca,'Xscale','log')

save('DATA_Critical_Fall.mat','Data');
save2pdf('../WriteUp/Figures/Critical_Fall.pdf',figH,600);


%Later on, compute the acceleration of the tip at the onset of sliding.





