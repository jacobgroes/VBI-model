function F = assmF(F,Fe,Te)
%********************************************************
% File: assmF.m
%   Assembles system matrix by adding element 
%   vector to existing global Vector. The system
%   vector may be a stiffness load, displacement 
%   vector etc.
% Syntax:
%   F = assmF(F,Fe,Te)
% Input:
%   F   : Initial global vector
%   Fe  : Element vector
%   Te  : Element topology array
% Output:
%   F   : Updated global vector
% Date:
%   Version 1.0    27.07.11
%********************************************************

% Define global address vector for element dofs                   
ig = edof(Te);
                                                                  
% Add element contributions to global load vector
F(ig) = F(ig) + Fe;