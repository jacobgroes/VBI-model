function [Mt,mb,kw,cw,kt,ct] = assemTrain(mwn,mbn,kwn,cwn,ll,lb,nt)
%**************************************************************************
% File: assemTrain.m
%   Assembles the modal system matrices governing the train and arranges
%   the wheel masses, spring stiffness and damping coefficients in diagonal
%   matrices.
% Syntax:
%   [Mt,mb,kw,cw,kt,ct] = assemTrain(mwn,mbn,kwn,cwn,ll,lb,nt)
% Input:
%   mwn : Train rigid mody mass properties
%   mbn : Wheel masses
%   kwn : Suspension spring stiffness
%   cwn : Suspension damping coefficient
%   ll  : CG distance to front and rear wheel
%   lb  : CG distance to left and right wheel
%   nt  : Number of trains
% Output:
%   Mt  : Train mass matrix
%   mb  : Wheel mass matrix
%   kw  : Suspension spring stiffness matrix
%   cw  : Suspension damping coefficient matrix
%   kt  : Train stiffness matrix
%   ct  : Train damping matrix
% Date:
%   Version 1.0    10.06.19
%**************************************************************************

for i=1:nt
% Train mass matrix
Mt(:,:,i) = diag(mwn(:,:,i));

mb(:,:,i) = diag(mbn(:,:,i));

kw(:,:,i) = diag(kwn(:,:,i));
         
cw(:,:,i) = diag(cwn(:,:,i));

Lf = ll(1,1,i); Lr = ll(2,1,i); bl = lb(1,1,i); br = lb(2,1,i);

kf1 = kw(1,1,i); kf2 = kw(3,3,i); kr1 = kw(2,2,i); kr2 = kw(4,4,i);
cf1 = cw(1,1,i); cf2 = cw(3,3,i); cr1 = cw(2,2,i); cr2 = cw(4,4,i);

% Train stiffness matrix
kt(:,:,i) = [kf1+kf2+kr1+kr2, (kf2+kr2)*br-(kf1+kr1)*bl, (kr1+kr2)*Lr-(kf1+kf2)*Lf;
            (kf2+kr2)*br-(kf1+kr1)*bl, (kf1+kr1)*bl^2+(kf2+kr2)*br^2, kf1*Lf*bl+kr2*Lr*br-kf2*Lf*br-kr1*Lr*bl;
            (kr1+kr2)*Lr-(kf1+kf2)*Lf, kf1*Lf*bl+kr2*Lr*br-kf2*Lf*br-kr1*Lr*bl, (kf1+kf2)*Lf^2+(kr1+kr2)*Lr^2];
            
% Train damping matrix
ct(:,:,i) = [cf1+cf2+cr1+cr2, (cf2+cr2)*br-(cf1+cr1)*bl, (cr1+cr2)*Lr-(cf1+cf2)*Lf;
            (cf2+cr2)*br-(cf1+cr1)*bl, (cf1+cr1)*bl^2+(cf2+cr2)*br^2, cf1*Lf*bl+cr2*Lr*br-cf2*Lf*br-cr1*Lr*bl;
            (cr1+cr2)*Lr-(cf1+cf2)*Lf, cf1*Lf*bl+cr2*Lr*br-cf2*Lf*br-cr1*Lr*bl, (cf1+cf2)*Lf^2+(cr1+cr2)*Lr^2];
end

