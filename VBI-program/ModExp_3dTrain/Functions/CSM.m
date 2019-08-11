function [M,K,C] = CSM(b,bh,mm,km,cm,mt,mb,kw,cw,kt,ct,ll,lb,nt,n)
%**************************************************************************
% File: CSM.m
%   Assembles the coupled system matrices M, K and C.
% Syntax:
%   [M,K,C] = CSM(b,bh,mm,km,cm,mt,mb,kw,cw,kt,ct,ll,lb,nt,n)
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
%   lb  : CG distance to left and right wheel
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
M = zeros(n+nt*3,n+nt*3);
K = zeros(n+nt*3,n+nt*3);
C = zeros(n+nt*3,n+nt*3);

o = zeros(1,1,nt);

% CG distance matrices
ll = [ll(1,1,:)    o         o           o
         o     -ll(2,1,:)    o           o
         o         o      ll(1,1,:)      o
         o         o         o       -ll(2,1,:)]; 

lb = [lb(1,1,:)    o         o           o
         o      lb(1,1,:)    o           o
         o         o     -lb(2,1,:)      o
         o         o         o       -lb(2,1,:)];  

% Modal system matrices of the bridge
Mm = mm;
Km = km;
Cm = cm;
for i=1:nt

% Definition of upper left square in system matrices
Mm = Mm + b(:,:,i)*mb(:,:,i)*b(:,:,i)' + bh(:,:,i)*mt(1,1,i)*bh(:,:,i)';
Km = Km + b(:,:,i)*kw(:,:,i)*b(:,:,i)';
Cm = Cm + b(:,:,i)*cw(:,:,i)*b(:,:,i)';  

% Coupling elements (train-bridge)
kc = [-sum(b(:,:,i)*kw(:,:,i),2) sum(b(:,:,i)*(kw(:,:,i)*lb(:,:,i)),2)...
    sum(b(:,:,i)*(kw(:,:,i)*ll(:,:,i)),2)];

cc = [-sum(b(:,:,i)*cw(:,:,i),2) sum(b(:,:,i)*(cw(:,:,i)*lb(:,:,i)),2)...
    sum(b(:,:,i)*(cw(:,:,i)*ll(:,:,i)),2)]; 

% Assembling of system matrices -------------------------------------------

% Coupled mass matrix
M(1:n,1:n) = Mm;
M(n+(3*i-2):n+3*i,n+(3*i-2):n+3*i) = mt(:,:,i);

% Coupled stiffness matrix
K(1:n,1:n) = Km;
K(n+(3*i-2):n+3*i,n+(3*i-2):n+3*i) = kt(:,:,i);
K(1:n,n+(3*i-2):n+3*i) = kc;
K(n+(3*i-2):n+3*i,1:n) = kc';

% Coupled damping matrix
C(1:n,1:n) = Cm;
C(n+(3*i-2):n+3*i,n+(3*i-2):n+3*i) = ct(:,:,i);  
C(1:n,n+(3*i-2):n+3*i) = cc;
C(n+(3*i-2):n+3*i,1:n) = cc';

end