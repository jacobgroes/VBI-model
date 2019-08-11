function [Me,me] = Mebeam(Xe,Ge,X3e,ip)
%********************************************************
% File: Mebeam.m
%   Creates the element mass matrix of a spatial
%   elastic beam element using numerical integration.
% Syntax:
%   Me = Mebeam(Xe,Ge,X3e,ip)
% Input:
%   Xe  : Nodal coordinates 
%   Ge : Element properties 
%   X3e : 3rd node coordinates
%    ip : Number of integration points / order
% Output:
%   Me:  Element mass matrix in global coordinates
%   me:  Element mass matrix in local coordinates
% Date:
%   Version 1.0    21.07.19
%********************************************************

% Form initial element (column) vector
a0 = (Xe(2,:)-Xe(1,:))';

% Element length
Le = sqrt(a0'*a0);

% Element transformation matrix
Ae = Aebeam(Xe,X3e);

% Gauss abscissae and weights
[r,w] = gaussint(ip);

% Initialize element mass matrix
me = zeros(12);

% Loop over integration points
for i = 1:length(r)
    
    % Normalized longitudinal coordinate
    x = 1/2*(1+r(i))*Le;
    
    % Element strain interpolation matrix
    Nu = Nubeam1(Xe,x);
    
     % Element material property matrix
    Je = Jebeam(Ge);
    
    % Element mass matrix
    me = me + w(i)*(Nu'*Je*Nu)*Le/2;
  
end

% Transform to global coordinates
Me = Ae'*me*Ae;

