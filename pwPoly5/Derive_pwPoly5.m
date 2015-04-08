% Derive_pwPoly5.m
%
% This script derives the equations that give the bezier coefficients for a
% curve, given the value, slope, and curvature at each endpoint.
%

clc; clear;

% Control Points:
n = 5;  %order of bezier curve
p = sym('p',[n+1,1]); assume(p,'real');

% Independent variable:
syms t 'real'

% Build up the equation for a 5th-order bezier curve:
x = sym(0);
for i=0:n
    tt = (t.^i).*(1-t).^(n-i);
    binom = nchoosek(n,i);
    x = x + binom*p(i+1)*tt;
end

% Now compute the derivaives:
dx = diff(x,t);
ddx = diff(dx,t);

% Evaluate at endpoints:
X0 = simplify(subs(x,t,0));
dX0 = simplify(subs(dx,t,0));
ddX0 = simplify(subs(ddx,t,0));
X1 = simplify(subs(x,t,1));
dX1 = simplify(subs(dx,t,1));
ddX1 = simplify(subs(ddx,t,1));

% User-Inputs:
syms x0 dx0 ddx0 x1 dx1 ddx1 'real'

% Constraint equations:
eqns = [...
    X0-x0;
    dX0-dx0;
    ddX0-ddx0;
    X1-x1;
    dX1-dx1;
    ddX1-ddx1];
    
% Solve for unknowns
[A,b] = equationsToMatrix(eqns,p);
pSoln = A\b;

% Substitute solution:
xSoln = simplify(subs(x,p,pSoln));
dxSoln = simplify(subs(dx,p,pSoln));
ddxSoln = simplify(subs(ddx,p,pSoln));

% Write solution to a matlab function:
matlabFunction(xSoln, dxSoln, ddxSoln,...
    'file','autoGen_pwPoly5.m',...
    'vars',{t,x0 dx0 ddx0 x1 dx1 ddx1},...
    'outputs',{'x','dx','ddx'});




