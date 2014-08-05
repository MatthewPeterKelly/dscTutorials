%Save_Figure_Script

%Variables that must be defined in the workspace:
% P
% iterationNum
% 

%Now, save the figure if desired:
if P.disp.saveIntermediatePlots
   BaseName = ['MS_Fig_Iter_' num2str(iterationNum)];
   FigureName = [BaseName '.pdf'];
   DirName = 'img/';  %Which subdirectory are the images in?   
   
   % %Force Figure Location (Good for extracting figures -> LaTeX)
   set(gcf,'position',800*[0,0,2,1])

   %This is a magical function that I got on file exchange
   %This should not be so hard to do in Matlab...
   save2pdf(FigureName,gcf,400)

   %Now, create the latex code if desired:
   if P.disp.createTex
       FileName = P.disp.TexFileName;
       fid = fopen(FileName,'a');  %Open file for appending
        fprintf(fid,'\n');
        fprintf(fid,'\\begin{figure} \n');
        fprintf(fid,'    \\centering \n');
        fprintf(fid,['    \\includegraphics[width = \\columnwidth]{' DirName FigureName '}    \n']);
        fprintf(fid,'    \\caption{}     \n');
        fprintf(fid,['    \\label{fig: ' BaseName '}     \n']);
        fprintf(fid,'\\end{figure} \n');
        fclose(fid);
   end
end