function writeSmoothAbs()
%
% FUNCTION:
%   This function is used to write the code for smoothAbs
%

MaxOrder = 8;  %Should be at least 1
N = MaxOrder + 1;

%% Set up the constraints:

% The value and slope should be 1 at the right transition point, and zero
% for any higher derivatives.
xRight = ones(1,N);
yRight = zeros(1,N);
yRight(1) = 1; yRight(2) = 1;

% The value and slope should be 1 at the right transition point, and zero
% for any higher derivatives.
xLeft = -ones(1,N);
yLeft = zeros(1,N);
yLeft(1) = 1; yLeft(2) = -1;

% All Constraints are expressed across all derivatives
dLevel = 0:N;


%% Begin writing the file:

fid = fopen('smoothAbs.m','w');

fprintf(fid,'function y = smoothAbs(x,alpha,order)\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% y = smoothAbs(x,alpha,order)\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% FUNCTION:\n');
fprintf(fid,'%%   This function is a smooth version of abs(). Smoothing is done using\n');
fprintf(fid,'%%   a piecewise polynomial, which is Nth order continuous at the\n');
fprintf(fid,'%%   transitions, where N is a value between 0 and 8, and input as ''order''\n');
fprintf(fid,'%%\n');
fprintf(fid,'%% INPUTS:\n');
fprintf(fid,'%%   x = a vector of matrix of real numbers\n');
fprintf(fid,'%%   alpha = smoothing parameter: -alpha<x<alpha will be smoothed\n');
fprintf(fid,'%%   order = the order of the smoothing (0=no smoothing)\n');
fprintf(fid,['%%           order must be in the set {0,1,..,' num2str(MaxOrder) '}\n']);
fprintf(fid,'%%\n');
fprintf(fid,'%% OUTPUTS:\n');
fprintf(fid,'%%   y = a smoothed version of abs(x)\n%%\n\n');
fprintf(fid,'switch order \n');
fprintf(fid,'    case 0 %% No Smoothing \n');
fprintf(fid,'        y=abs(x); \n');

for i=2:N
    x = [xLeft(1:i),xRight(1:i)];
    y = [yLeft(1:i),yRight(1:i)];
    d = [dLevel(1:i),dLevel(1:i)];
    [~,pSym] = SolveCoeff(x,y,d);
    str = 'p = [';
    for j=2:length(pSym)  %Leading coefficient is always zero   
        if j==length(pSym)
            str = [str char(pSym(j)) ']; \n']; %#ok<*AGROW>
        else
            str = [str char(pSym(j)) ','];
        end
    end
    fprintf(fid,['    case ' num2str(i-1) ' \n']);
    fprintf(fid,'        y=x; \n');
    fprintf(fid,'        c1 = x < -alpha; \n');
    fprintf(fid,'        c2 = x > alpha; \n');
    fprintf(fid,'        y(c1)=-x(c1); \n');
    fprintf(fid,['        ' str]);
    fprintf(fid,'        idx = ~c1&~c2;\n');
    fprintf(fid,'        y(idx)=alpha*polyval(p,x(idx)/alpha); \n');
end

fprintf(fid,'    otherwise \n');
fprintf(fid,'        error(''Order not supported'');\n');
fprintf(fid,'end \n');

fprintf(fid,'end \n');


fclose(fid);
end

