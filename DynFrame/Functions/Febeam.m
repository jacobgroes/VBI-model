function Fe = Febeam(Xe,X3e,p,ip)
%********************************************************
% File: Febeam.m
%   Creates the element nodal load vector of a spatial 
%   elastic beam element using numerical integration.
%   The nodal loads are equivalent with those for a
%   beam element subjected to a linearly varying load in
%   the x-,y- and z-direction.
% Syntax:
%   Fe = Febeam(Xe,X3e,p,ip)
% Input:
%   Xe  : Nodal coordinates, Xe = [ x1 y1 z1; x2 y2 z2 ]
%   X3e : 3rd node coordinates, X3e = [x3 y3 z3]
%    p  : Element distributed load array
%         p = [ el px1 px2 py1 py2 pz1 pz2 ]
%    ip : Number of integration points / order
% Output:
%   Fe:  Element load vector in global coordinates
% Date:
%   Version 1.0    27.07.11
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Element transformation matrix
Ae = Aebeam(Xe,X3e);

% Gauss abscissae and weights
[r,w] = gaussint(ip);

% Initialise element load vector
fe = zeros(12,1);

% Loop over integration points
for i = 1:length(r)
    
    % Normalized longitudinal coordinate
    s = 1/2*(1+r(i));
    x = s*Le;
    
    % Load intensities at Gauss point
    pe = pebeam(Xe,p,x);
    
    % Element shape function matrix
    Ne = Nubeam(Xe,s);

    % Element load vector in local coordinates
    fe = fe + w(i)*(Ne'*pe)*Le/2;

end

% Transform to global coordinates
Fe = Ae'*fe;
