function [Mt,mbn,kwn,cwn,kt,ct] = assemTrain(mw,mb,kw,cw,l,nt)
%**************************************************************************
% File: assemTrain.m
%   Assembles the modal system matrices governing the train and arranges
%   the wheel masses, spring stiffness and damping coefficients in diagonal
%   matrices.
% Syntax:
%   [Mt,mbn,kwn,cwn,kt,ct] = assemTrain(mw,mb,kw,cw,l,nt)
% Input:
%   mw  : Train rigid mody mass properties
%   mb  : Wheel masses
%   kw  : Suspension spring stiffness
%   cw  : Suspension damping coefficient
%   l   : CG distance to front and rear wheel
%   nt  : Number of trains
% Output:
%   Mt  : Train mass matrix
%   mbn : Wheel mass matrix
%   kwn : Suspension spring stiffness matrix
%   cwn : Suspension damping coefficient matrix
%   kt  : Train stiffness matrix
%   ct  : Train damping matrix
% Date:
%   Version 1.0    10.06.19
%**************************************************************************

for i=1:nt
% Train mass matrix
Mt(:,:,i) = diag(mw(:,:,i));

mbn(:,:,i) = diag(mb(:,:,i));

% Train stiffness matrix
kt(:,:,i) = [(kw(1,:,i)+kw(2,:,i)) -(kw(1,:,i)*l(1,1,i)-kw(2,:,i)*l(2,1,i));
            -(kw(1,:,i)*l(1,1,i)-kw(2,:,i)*l(2,1,i)) (kw(1,:,i)*l(1,1,i)^2+kw(2,:,i)*l(2,1,i)^2)];

% Train damping matrix
ct(:,:,i) = [(cw(1,:,i)+cw(2,:,i)) -(cw(1,:,i)*l(1,1,i)-cw(2,:,i)*l(2,1,i));
            -(cw(1,:,i)*l(1,1,i)-cw(2,:,i)*l(2,1,i)) (cw(1,:,i)*l(1,1,i)^2+cw(2,:,i)*l(2,1,i)^2)];
        
kwn(:,:,i) = [kw(1,:,i) 0
             0  kw(2,:,i)];
         
cwn(:,:,i) = [cw(1,:,i) 0
             0  cw(2,:,i)];
end

