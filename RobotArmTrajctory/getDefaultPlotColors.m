function Color = getDefaultPlotColors()

n = 7;  %Number of unique colors in matlab's default line color pallet

Color = zeros(n,3);
hFig = figure('visible','off'); 
hold on;

for i=1:n
   hLine = plot(rand(1,2),rand(1,2));
   Color(i,:) = get(hLine,'Color');
end

close(hFig);

end