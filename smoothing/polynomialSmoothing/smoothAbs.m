function y = smoothAbs(x,alpha,order)
%
% y = smoothAbs(x,alpha,order)
%
% FUNCTION:
%   This function is a smooth version of abs(). Smoothing is done using
%   a piecewise polynomial, which is Nth order continuous at the
%   transitions, where N is a value between 0 and 8, and input as 'order'
%
% INPUTS:
%   x = a vector of matrix of real numbers
%   alpha = smoothing parameter: -alpha<x<alpha will be smoothed
%   order = the order of the smoothing (0=no smoothing)
%           order must be in the set {0,1,..,8}
%
% OUTPUTS:
%   y = a smoothed version of abs(x)
%
switch order 
    case 0 % No Smoothing 
        y=abs(x); 
    case 1 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [1/2,0,1/2]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 2 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [-1/8,0,3/4,0,3/8]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 3 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [1/16,0,-5/16,0,15/16,0,5/16]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 4 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [-5/128,0,7/32,0,-35/64,0,35/32,0,35/128]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 5 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [7/256,0,-45/256,0,63/128,0,-105/128,0,315/256,0,63/256]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 6 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [-21/1024,0,77/512,0,-495/1024,0,231/256,0,-1155/1024,0,693/512,0,231/1024]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 7 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [33/2048,0,-273/2048,0,1001/2048,0,-2145/2048,0,3003/2048,0,-3003/2048,0,3003/2048,0,429/2048]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 8 
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
        y(c1)=-x(c1); 
        p = [-429/32768,0,495/4096,0,-4095/8192,0,5005/4096,0,-32175/16384,0,9009/4096,0,-15015/8192,0,6435/4096,0,6435/32768]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    otherwise 
        error('Order not supported');
end 
end 
