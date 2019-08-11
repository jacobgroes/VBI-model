function [F] = assemLoad(Fi,b,mw,g,Mt,nt,n,i)
%**************************************************************************
% File: assemLoad.m
%   Builds modal load vector from stationary dynamic loads and 
%   gravitational load from the train
% Syntax:
%   [F] = assemLoad(Fi,b,mw,g,Mt,nt,n,i)
% Input:
%   Fi  : Stationary dynamic loads
%   b   : Modal interaction vector
%   mw  : Wheel mass matrix
%   g   : Gravitational acceleration
%   Mt  : Train mass matrix
%   nt  : Number of trains
%   n   : Number of modes included in analysis
%   i   : time-step
% Output:
%   F   : Modal load array
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
Fm = Fi(:,i);

gc = zeros(2,1); gc(1,1,:) = g; 

for j=1:nt
% Modal loads 
F(1:n,1) = Fm-sum(b(:,:,j)*mw(:,:,j)*g,2);
F(n+(2*j-1):n+2*j,1) = -sum(Mt(:,:,j)*gc,2);
end