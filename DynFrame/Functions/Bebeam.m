function Be = Bebeam(Xe,x)
%********************************************************
% File: Bebeam.m
%   Creates the element strain-displacement matrix of a
%   spatial elastic beam element.
% Syntax:
%   Be = Bebeam(Xe,x)
% Input:
%   Xe  : Nodal coordinates, Xe = [ x1 y1 z1; x2 y2 z2 ]
%    x  : Longitudinal coordinate to compute strain
% Output:
%   Be  :  Element strain-displacement matrix
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Normalised length
s = x/Le;

% Element strain-displacement matrix
Be = 1/Le * ...
[ -1           0           0  0         0         0 1           0           0  0         0         0    % du/dx
   0 6*(2*s-1)/Le          0  0         0 2*(3*s-2) 0 6*(1-2*s)/Le          0  0         0 2*(3*s-1)    % d^2v/dx^2
   0           0 6*(2*s-1)/Le 0  2*(2-3*s)        0 0           0 6*(1-2*s)/Le 0 2*(1-3*s)         0    % d^2w/dx^2
   0           0           0 -1         0         0 0           0           0  1         0         0 ]; % dphi/dx
