function [Ae,He] = Aebeam(Xe,X3e)
%********************************************************
% File: Aebeam.m
%   Creates the element transformation matrix and rotation
%   matrix for the transformation between local- and 
%   global coordinates for a spatial beam element.
% Syntax:
%   Ae = Aebeam(Xe,X3e)
% Input:
%   Xe  : Element nodal coordinate array,
%         Xe = [ x1 y1 z1; x2 y2 z2 ]
%   X3e : Element 3rd node coordinate array,
%         X3e = [x3 y3 z3]
% Output:
%   Ae  : Element transformation matrix
%   He  : Element rotation matrix
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Form initial element (column) vector   
a0 = (Xe(2,:)-Xe(1,:))';
                                         
% Element length                         
Le = sqrt(a0'*a0);                       

% Element orientation arrays
V12 = (Xe(2,:)-Xe(1,:));
Vx  = V12/Le;
V13 = (X3e(1,:)-Xe(1,:));
Vz  = cross(V13,V12);
Lz  = sqrt(Vz*Vz');
Vz  = Vz/Lz;
Vy  = cross(Vz,Vx);

% Direction cosine rotation matrix
He = [ Vx ; Vy ; Vz ];

% 3-by-3 zero array
O = zeros(3);

% Element transformation matrix
Ae = [ He O O O ; O He O O ; O O He O ; O O O He ];
