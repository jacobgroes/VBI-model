function K = assmK(K,Ke,Te)
%********************************************************
% File: assmK.m
%   Assembles system matrix by adding element 
%   matrix to existing global matrix. The system
%   matrix may be a stiffness matrix, mass matrix,
%   conductivity matrix, etc.
% Syntax:
%   K = assmK(K,Ke,Te)
% Input:
%   K   : Initial global matrix
%   Ke  : Element matrix
%   Te  : Element topology array
% Output:
%   K   : Updated global matrix
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Define global address vector for element dofs
ig = edof(Te);

% Add element contributions to global stiffness matrix
K(ig,ig) = K(ig,ig) + Ke;


     
