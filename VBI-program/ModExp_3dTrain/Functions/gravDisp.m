function [x0] = gravDisp(F,nt,n,kt,x0)
%**************************************************************************
% File: gravDisp.m
%   Calculates static displacement of each train from gravitational load
% Syntax:
%   gravDisp(F,nt,n,kt,x0)
% Input:
%   F   : Load vector
%   nt  : Number of trains
%   n   : Number of modes included in analysis
%   kt  : Train stiffness matrix
%   x0  : Initialised initial displacements
% Output:
%   x0  : Initial displacements 
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
for j=1:nt
x0(n+(3*j-2):n+3*j,1) = kt(:,:,j)\F(n+(3*j-2):n+3*j,1);
end