function Nu = Nubeam1(Xe,x)
%********************************************************
% File: Nubeam.m
%   Creates the element interpolation matrix of a
%   spatial elastic beam element.
% Syntax:
%   Nu = Nubeam1(Xe,x)
% Input:
%   Xe : Nodal coordinates, Xe = [ x1 y1 z1; x2 y2 z2 ]
%    x : Coordinate to compute displacement
% Output:
%   Nu : Element interpolation matrix
% Date:
%   Version 1.0    21.07.19
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Normalised length
s = x/Le;

% Interpolation matrix
Nu = [ 1-s             0             0 0                 0                0  s            0            0 0             0            0
         0 2*s^3-3*s^2+1             0 0                 0  Le*(s^3-2*s^2+s) 0 -2*s^3+3*s^2            0 0             0 Le*(s^3-s^2)
         0             0 2*s^3-3*s^2+1 0 -Le*(s^3-2*s^2+s)                0  0            0 -2*s^3+3*s^2 0 -Le*(s^3-s^2)            0
         0             0             0 1-s               0                0  0            0            0 s             0            0];
     
     