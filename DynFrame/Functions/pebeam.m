function pe = pebeam(Xe,p,x)
%********************************************************
% File: pebeam.m
%   Evaluates the element loads along a spatial elastic
%   beam element subjected to distributed loads.
% Syntax:
%   pe = pebeam(Xe,p,x)
% Input:
%   Xe  : Nodal coordinates, Xe = [ x1 y1 z1; x2 y2 z2 ]
%    p  : Element distributed load array
%         pe = [ el px1 px2 py1 py2 pz1 pz2 ]
%    x  : Longitudinal coordinate to compute load
% Output:
%   pe  : Element distributed load vector evaluated at
%         the coordinate x
% Date:
%   Version 1.0    27.07.11
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Initialisation
pe = zeros(3,1);

% Loop over load components
for i = 1:3
    % Evaluate load at x
    pe(i) = (p(2*i+1)-p(2*i))/Le*x + p(2*i);
end
