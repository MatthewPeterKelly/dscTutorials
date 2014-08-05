function writeSmoothRamp()
%
% FUNCTION: 
% This function is used to write the code for smoothRamp 
%

MaxOrder = 8;  %Should be at least 1
N = MaxOrder + 1;

%% Set up the constraints:

% The value and slope should be 1 at the right transition point, and zero
% for any higher derivatives.
xRight = ones(1,N);
yRight = zeros(1,N);
yRight(1) = 1; yRight(2) = 1;

% The value and all of its derivatives should be equal to zero at the left
% transition point.
xLeft = -ones(1,N);
yLeft = zeros(1,N);

% All Constraints are expressed across all derivatives
dLevel = 0:N;


%% Begin writing the file:

fid = fopen('smoothRamp.m','w');

fprintf(fid,'function y = smoothRamp(x,alpha,order)\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% y = smoothRamp(x,alpha,order)\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% FUNCTION:\n');
fprintf(fid,'%%This function is a smooth version of the standard ramp function, which is:\n');
fprintf(fid,'%%   ramp(x) = 0 ; (x<0)\n');
fprintf(fid,'%%   ramp(x) = x ; (x>=0)\n');
fprintf(fid,'%% Smoothing is done using a polynomial transition, the degree of smoothness\n');
fprintf(fid,'%% at the transitions is set using order.\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% INPUTS:\n');
fprintf(fid,'%%   x = a vector of matrix of real numbers\n');
fprintf(fid,'%%   alpha = smoothing parameter: -alpha<x<alpha will be smoothed\n');
fprintf(fid,'%%   order = the order of the smoothing (0=no smoothing)\n');
fprintf(fid,['%%           order must be in the set {0,1,..,' num2str(MaxOrder) '}\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% OUTPUTS:\n');
fprintf(fid,'%%   y(x<-alpha) = 0;\n');
fprintf(fid,'%%   y(x>alpha) = x;\n');
fprintf(fid,'%%   y(else) = smooth_transition\n');
fprintf(fid,'%%\n');

fprintf(fid,'        y=x; \n');
fprintf(fid,'        c1 = x < -alpha; \n');
fprintf(fid,'        c2 = x > alpha; \n');

fprintf(fid,'switch order \n');
fprintf(fid,'    case 0 %% No Smoothing \n');
fprintf(fid,'        y(x<0)=0; \n');

for i=2:N
    %Collect the constraints:
    x = [xLeft(1:i),xRight(1:i)];
    y = [yLeft(1:i),yRight(1:i)];
    d = [dLevel(1:i),dLevel(1:i)];
    %Solve for the coefficients
    [~,pSym] = SolveCoeff(x,y,d);
    %Parse the symbolic output to a string
    str = 'p = [';
    for j=2:length(pSym)   %NOTE - pSym(1) is always zero
        if j==length(pSym)
            str = [str char(pSym(j)) ']; \n']; %#ok<*AGROW>
        else
            str = [str char(pSym(j)) ','];
        end
    end
    %Write the function for order (i-1)
    fprintf(fid,['    case ' num2str(i-1) ' \n']);
    fprintf(fid,'        y=x; \n');
    fprintf(fid,'        y(c1)=0; \n');
    fprintf(fid,['        ' str]);
    fprintf(fid,'        idx = ~c1&~c2;\n');
    fprintf(fid,'        y(idx)=alpha*polyval(p,x(idx)/alpha); \n');
end

%Print the end of the function
fprintf(fid,'    otherwise \n');
fprintf(fid,'        error(''Order not supported'');\n');
fprintf(fid,'end \n');

fprintf(fid,'end \n');
fclose(fid);

end

