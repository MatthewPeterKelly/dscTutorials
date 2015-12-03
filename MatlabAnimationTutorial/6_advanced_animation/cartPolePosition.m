function pos = cartPolePosition(z,p)
% pos = cartPolePosition(z,p)
%
% This function computes the position of the cart and the pole, given the
% state of the system
%
% INPUTS:
%   z = [4, n] = [x;q;dx;dq] = state of the system
%   p = parameter struct
%       .g = gravity
%       .m1 = cart mass
%       .m2 = pole mass
%       .l = pendulum length
% OUTPUTS:
%   pos = [4, n] = [x1;y1;x2;y2]; = position of [cart; pole]
%

%%%% unpack the state
x = z(1,:);   %Cart position (Not used in dynamics)
q = z(2,:);   % pendulum (pole) angle, measure from gravity vector

%%%% Unpack the physical parameters
l = p.l;  %Pendulum length

%%%% Position of the cart:
x1 = x;
y1 = zeros(size(x));

%%%% Position of the pole:
x2 = x1 + l*sin(q);
y2 = y1 - l*cos(q);

%%%% Pack up position vector:
pos = [x1;y1;x2;y2];

end