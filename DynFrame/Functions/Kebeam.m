function [Ke,ke] = Kebeam(Xe,Ge,X3e,ip)
%********************************************************
% File: Kebeam.m
%   Creates the element stiffness matrix of a spatial
%   elastic beam element using numerical integration.
% Syntax:
%   Ke = Kebeam(Xe,Ge,X3e,ip)
% Input:
%   Xe  : Nodal coordinates, Xe = [ x1 y1 z1; x2 y2 z2 ]
%   Ge  : Element properties, Ge = [ EA EIy EIz GK ]
%   X3e : 3rd node coordinates, X3e = [x3 y3 z3]
%    ip : Number of integration points / order
% Output:
%   Ke:  Element stiffness matrix in global coordinates
%   ke:  Element stiffness matrix in local coordinates
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Element transformation matrix
Ae = Aebeam(Xe,X3e);

% Gauss abscissae and weights
[r,w] = gaussint(ip);

% Initialize element stiffness matrix
ke = zeros(12);

% Loop over integration points
for i = 1:length(r)
    
    % Normalized longitudinal coordinate
    x = 1/2*(1+r(i))*Le;
    
    % Element strain interpolation matrix
    Be = Bebeam(Xe,x);
    
     % Element material property matrix
    De = Debeam(Ge);
    
    % Element stiffness matrix
    ke = ke + w(i)*(Be'*De*Be)*Le/2;
  
end

% Transform to global coordinates
Ke = Ae'*ke*Ae;
