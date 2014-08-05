function AxB = Cross(A,B)
%This is a custom implementation of the Cross product operator;
%A and B must be (2x1) symbolic matricies, and AxB is a symbolic variable.
%It assumes that A and B are planar (with components i and j) and that AxB
%is the value of the number out of plane (in the k direction)

AxB = A(1)*B(2) - A(2)*B(1);

end