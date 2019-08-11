function Je = Jebeam(Ge)
%********************************************************
% File: Jebeam.m
%   Creates the matrix of element mass 
%   properties of a spatial elastic beam element.
% Syntax:
%   Je = Jebeam(Je)
% Input:
%   Ge  : Element properties
% Output:
%   Je:  Matrix of element mass properties
% Date:
%   Version 1.0    21.07.19
%********************************************************

% Element mass properties
rho = Ge(7); A = Ge(2); Ip = Ge(8);

% Element mass matrix
Je = rho*[A  0  0  0
          0  A  0  0
          0  0  A  0
          0  0  0  Ip ];
    