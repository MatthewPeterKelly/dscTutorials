function [F, Fz] = computeGradientOfBackSlash(A,B,Az,Bz,nx,nz)
% [F, Fz] = computeGradientOfBackSlash(A,B,Az,Bz,n)
%
% Suppose that you have the following vector function:
%   F(z) = A(z)\B(z);
%
% This function computes F and dF/dz for vectorized inputs
%
% INPUTS:
%   A = [nx*nx,nt] = mass matrix, reshaped
%   B = [nx, nt] = generalized force, reshaped
%   Az = [nx*nx*nz,nt] = gradient of mass matrix, reshaped
%   Bz = [nx*nz,nt] = gradient of generalized force, reshaped
%
% OUTPUTS:
%   F = [nx, nt] = solution
%   Fz = [nx, nz, nt] = gradient of solution
%
% NOTES:
%   How to reshape vectors:
%       A = reshape(A,nx*nx,1)
%       Az = reshape(jacobian(reshape(A,nx*nx,1),z),nx*nx*nz,1);
%       Bz = reshape(jacobian(b,z),nx*nz,1);
%
%   Derivative of matrix inverse:
%       http://www.atmos.washington.edu/~dennis/MatrixCalculus.pdf
%

nt = size(A,2);

F = zeros(nx,nt);
Fz = zeros(nx,nz,nt);

a = zeros(nx,nx);
b = zeros(nx,1);
az = zeros(nx,nx,nz);
bz = zeros(nx,nz);
for i=1:nt
    a(:) = A(:,i);
    b(:) = B(:,i);
    F(:,i) = a\b;
    az(:) = Az(:,i);
    bz(:) = Bz(:,i);
    for j=1:nz
        Fz(:,j,i) = a\(-az(:,:,j)*F(:,i) + bz(:,j));
    end
end


end