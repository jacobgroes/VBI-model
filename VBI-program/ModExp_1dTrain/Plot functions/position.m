function [posf] = position(N,np,initial,vt,tt,X,T)
%**************************************************************************
% File: position.m
%   Creates a matrix discribing the travelled distance by each contact 
%   point at each time-step.
% Syntax:
%   posf = position(N,np,initial,vt,tt,nt,X)
% Input:
%   N   : Number of time increments
%   np  : Number of contact points
%   initial: Initial positions of all wheels
%   vt  : Constant train velocity
%   tt  : Time increment vector
%   X   : Node coordinate array
%   T   : Topology array
% Output:
%   posf: Position matrix
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
posf = zeros(np,N+1);
for i=1:N+1
    posf(:,i) = initial + vt*tt(i) + X(T(1,1),1);
end