function De = Debeam(Ge)
%********************************************************
% File: Debeam.m
%   Creates the element constitutive matrix of a
%   spatial elastic beam element.
% Syntax:
%   De = Debeam(Ge)
% Input:
%   Ge  : Element properties, Ge = [ EA EIy EIz GK ]
% Output:
%   De:  Element constitutive matrix
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Element properties
EA = Ge(1)*Ge(2);
EIz = Ge(1)*Ge(3);
EIy = Ge(1)*Ge(4);
GK = Ge(5)*Ge(6);
 

% Element constitutive matrix
De = [ EA  0   0   0
        0 EIz  0   0
        0  0  EIy  0
        0  0   0  GK ];