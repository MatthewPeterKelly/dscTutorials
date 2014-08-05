function y = smoothRamp(x,alpha,order)
%
% y = smoothRamp(x,alpha,order)
%
% FUNCTION:
%This function is a smooth version of the standard ramp function, which is:
%   ramp(x) = 0 ; (x<0)
%   ramp(x) = x ; (x>=0)
% Smoothing is done using a polynomial transition, the degree of smoothness
% at the transitions is set using order.
%
% INPUTS:
%   x = a vector of matrix of real numbers
%   alpha = smoothing parameter: -alpha<x<alpha will be smoothed
%   order = the order of the smoothing (0=no smoothing)
%           order must be in the set {0,1,..,8}
%
% OUTPUTS:
%   y(x<-alpha) = 0;
%   y(x>alpha) = x;
%   y(else) = smooth_transition
%
        y=x; 
        c1 = x < -alpha; 
        c2 = x > alpha; 
switch order 
    case 0 % No Smoothing 
        y(x<0)=0; 
    case 1 
        y=x; 
        y(c1)=0; 
        p = [1/4,1/2,1/4]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 2 
        y=x; 
        y(c1)=0; 
        p = [-1/16,0,3/8,1/2,3/16]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 3 
        y=x; 
        y(c1)=0; 
        p = [1/32,0,-5/32,0,15/32,1/2,5/32]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 4 
        y=x; 
        y(c1)=0; 
        p = [-5/256,0,7/64,0,-35/128,0,35/64,1/2,35/256]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 5 
        y=x; 
        y(c1)=0; 
        p = [7/512,0,-45/512,0,63/256,0,-105/256,0,315/512,1/2,63/512]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 6 
        y=x; 
        y(c1)=0; 
        p = [-21/2048,0,77/1024,0,-495/2048,0,231/512,0,-1155/2048,0,693/1024,1/2,231/2048]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 7 
        y=x; 
        y(c1)=0; 
        p = [33/4096,0,-273/4096,0,1001/4096,0,-2145/4096,0,3003/4096,0,-3003/4096,0,3003/4096,1/2,429/4096]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    case 8 
        y=x; 
        y(c1)=0; 
        p = [-429/65536,0,495/8192,0,-4095/16384,0,5005/8192,0,-32175/32768,0,9009/8192,0,-15015/16384,0,6435/8192,1/2,6435/65536]; 
        idx = ~c1&~c2;
        y(idx)=alpha*polyval(p,x(idx)/alpha); 
    otherwise 
        error('Order not supported');
end 
end 
