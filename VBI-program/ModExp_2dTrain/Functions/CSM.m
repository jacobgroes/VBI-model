function [M,K,C] = CSM(b,bh,mm,km,cm,mt,mb,kw,cw,kt,ct,ll,nt,n)
%**************************************************************************
% File: CSM.m
%   Assembles the coupled system matrices M, K and C.
% Syntax:
%   [M,K,C] = CSM(b,bh,mm,km,cm,mt,mb,kw,cw,kt,ct,ll,nt,n)
% Input:
%   b   : Modal interaction matrix
%   bh  : Horizontal modal interaction matrix 
%   mm  : Bridge modal mass matrix
%   km  : Bridge modal stiffness matrix
%   cm  : Bridge modal damping matrix
%   mt  : Train mass matrix
%   mb  : Train wheel masses
%   kw  : Train suspension spring stiffness
%   cw  : Train suspension damping coefficients
%   kt  : Train stiffness matrix
%   ct  : Train damping matrix
%   ll  : CG distance to front and rear wheel
%   nt  : Number of trains
%   n   : Number of modes included in analysis
% Output:
%   M   : Coupled mass matrix
%   K   : Coupled stiffness matrix
%   C   : Coupled damping matrix
% Date:
%   Version 1.0    10.06.19
%**************************************************************************

% System matrix initialisation
M = zeros(n+nt*2,n+nt*2);
K = zeros(n+nt*2,n+nt*2);
C = zeros(n+nt*2,n+nt*2);

% Matrix of zeros
o = zeros(1,1,nt);

% Distance matrix
ll = [ll(1,1,:)  o
     o -ll(2,1,:)]; 

% Bridge modal system matrices initialisation
Mm = mm;
Km = km;
Cm = cm;
for i=1:nt

% Upper left square of system matrices
Mm = Mm + b(:,:,i)*mb(:,:,i)*b(:,:,i)'+ bh(:,:,i)*mt(:,:,i)*bh(:,:,i)';
Km = Km + b(:,:,i)*kw(:,:,i)*b(:,:,i)';
Cm = Cm + b(:,:,i)*cw(:,:,i)*b(:,:,i)';

% Coupling elements (train-bridge)
kc = [-sum(b(:,:,i)*kw(:,:,i),2) sum(b(:,:,i)*(kw(:,:,i)*ll(:,:,i)),2)];
cc = [-sum(b(:,:,i)*cw(:,:,i),2) sum(b(:,:,i)*(cw(:,:,i)*ll(:,:,i)),2)]; 

% Assembling of system matrices

% Coupled mass matrix
M(1:n,1:n) = Mm;
M(n+(2*i-1):n+2*i,n+(2*i-1):n+2*i) = mt(:,:,i);

% Coupled stiffness matrix
K(1:n,1:n) = Km;
K(n+(2*i-1):n+2*i,n+(2*i-1):n+2*i) = kt(:,:,i);
K(1:n,n+(2*i-1):n+2*i) = kc;
K(n+(2*i-1):n+2*i,1:n) = kc';

% Coupled damping matrix
C(1:n,1:n) = Cm;
C(n+(2*i-1):n+2*i,n+(2*i-1):n+2*i) = ct(:,:,i);  
C(1:n,n+(2*i-1):n+2*i) = cc;
C(n+(2*i-1):n+2*i,1:n) = cc';

end
