function [K,F] = setbound(K,F,C,dof)
%***************************************************
% File: setbound.m
%   Enforces the boundary conditions on the system
%   stiffness matrix and load vector according to the
%   penalty approach.
% Syntax:
%  [K,F] = setbound(K,F,C,dof)
% Input:
%   K : Original global stiffness matrix
%   F : Original global load vector
%   C : Constraint array given as C = [ node dof u ]
%   P : Original global load vector
% dof : No. of dof pr. node
% Output:
%  K : Global stiffness matrix modified to include
%      spring rigidities
%  F : Global load vector modified to include
%      spring loads
% Date:
%   Version 1.0    01.09.10
%***************************************************

% Set value of spring stiffness
ks = 10^6*max(abs(diag(K)));

% Introduce constraining spring stiffness and loads
for i = 1:size(C,1)
    j = (C(i,1)-1)*dof + C(i,2);
    K(j,j) = K(j,j) + ks;
    F(j)   = F(j) + ks*C(i,size(C,2));
end
