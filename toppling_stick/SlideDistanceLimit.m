%SlideDistanceLimit

%w = sqrt(2*g / 3*L);
%cos(th) = 2/3;
%sin(th) = sqrt(1-cos(th)^2);
%
%rg = [L*sin(th); L*cos(th)];
%
% Let L=1, g = 1
%

costh = 2/3;
sinth = sqrt(1-costh^2);
w = sqrt(2/3);

rg = [-sinth; costh];
drg = [-w*costh; -w*sinth];

%0 = rg(2) + drg(2)*t + -0.5*t^2
P = [-0.5,drg(2),rg(2)];
t = max(roots(P));

d = rg(1) + drg(1)*t;

SlipDist = d+1;

%%%%%%%%%%%%%%

syms L g

L=sym(1);
g = sym(1);

costh = sym(2/3);
sinth = sqrt(sym(5/9));
w = sqrt((2*g)/(3*L));

rg = [-L*sinth; L*costh];
drg = -L*w*[costh; sinth];
a = -g/2;
b = drg(2);
c = rg(2);
t = simplify((-b - sqrt(b^2-4*a*c))/(2*a));

d = simplify(rg(1) + drg(1)*t);

SlipDist = simplify(d + L);

pretty(SlipDist)
SlipDist
double(SlipDist)


