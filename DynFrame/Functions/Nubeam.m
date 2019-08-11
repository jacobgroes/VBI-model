function Nu = Nubeam(Xe,s)
%********************************************************
% File: Nubeam.m
%   Creates the element interpolation matrix of a
%   spatial elastic beam element.
% Syntax:
%   Nu = Nubeam(Xe,x)
% Input:
%   Xe : Nodal coordinates, Xe = [ x1 y1 z1; x2 y2 z2 ]
%    s : Normalised coordinate to compute displacement
% Output:
%   Nu : Element interpolation matrix
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Interpolation matrix
Nu = [ 1-s             0             0 0                 0                0  s            0            0 0             0            0
         0 2*s^3-3*s^2+1             0 0                 0  Le*(s^3-2*s^2+s) 0 -2*s^3+3*s^2            0 0             0 Le*(s^3-s^2)
         0             0 2*s^3-3*s^2+1 0 -Le*(s^3-2*s^2+s)                0  0            0 -2*s^3+3*s^2 0 -Le*(s^3-s^2)            0 ];