function [pNum,pSym] = SolveCoeff(x,y,dLevel)
%
% [pNum,pSym] = SolveCoeff(x,y,dLevel)
%
%FUNCTION:
% This function is used to symbolically find the coefficients of a
% polynomial that precisely meets the constraints described in the
% arguments.
%
%INPUTS:
% x = vector of input constraints
% y = vector of output constraints
% dLevel = vector giving the derivitive level of each constraint
% 
% NOTE: length(x) == length(y) == length(dLevel)
%
% Example: Find a parabola that has zero slope at x=0 and passes through
% the points (0,2) and (4,0):
%           x = [0,0,4];
%           y = [0,2,0];   
%      dLevel = [1,0,0];
%
%      returns pSym = [-1/8,0,2];
%
%OUTPUTS:
% pNum = a vector of coefficients to be evaluated with polyval
% pSym = a symbolic representation of the same vector;

N = length(x);

p = sym('p',[N,1]);  %symbolic constraints to solve for

c = sym('c',[N,1]);  %Store the constraints here

for i=1:N
   c(i) = symConstraint(x(i),y(i),dLevel(i),p);
end

%Dealing with stupid syntax for solve.m
expr = [];
vars = [];
for i=1:N
    if i==N
        expr = [expr 'c(' num2str(i) ')']; %#ok<*AGROW>
        vars = [vars 'p(' num2str(i) ')'];
    else
        expr = [expr 'c(' num2str(i) '),'];
        vars = [vars 'p(' num2str(i) '),'];
    end
end

pSoln = eval(['solve(' expr ',' vars ');']); %#ok<*NASGU>

pNum = zeros(N,1);
pSym = sym('pSym',[N,1]);

for i=1:N
    pSym(i) = eval(['pSoln.p' num2str(i)]);
    pNum(i) = double(pSym(i));
end


end



function expr = symConstraint(x,y,dLevel,p)

%x = input value for constraint
%y = output value for constraint
%dLevel = what derivitive is the constraint on
%p = symbolic coefficients of the polynomial

xSym = sym('x');

N = length(p);

f = sym(0);
for i=1:N
   f = f + p(i)*xSym^(N-i);
end

for i=1:dLevel
   f = diff(f,xSym);
end

f = subs(f,xSym,x);
expr = f-y;

end