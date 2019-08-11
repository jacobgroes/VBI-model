function [r,w] = gaussint(ip)
%********************************************************
% File: gaussint.m
%   Contains the abscissae and weights used in Gauss
%   numerical integration of order 1,2,3 or 4.
% Syntax:
%   [r,w] = gaussint(ip)
% Input:
%    ip : Number of integration points / order
% Output:
%    r  : Gauss abscissae vector
%    w  : Gauss weight vector
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Gauss abscissae and weights:

% 1 point rule
if ip == 1
r = [0];  w = [2];

% 2 point rule
elseif ip == 2
r = [-1 1]/sqrt(3);  w = [1 1];

% 3 point rule
elseif ip == 3
r = [-1 0 1]*sqrt(3/5);  w = [5 8 5]/9;

% 4 point rule
else
a = sqrt(1.2);
r = [-((3+2*a)/7)^(1/2) -((3-2*a)/7)^(1/2) ((3-2*a)/7)^(1/2)...
        ((3+2*a)/7)^(1/2)];
w = [1/2-1/(6*a) 1/2+1/(6*a) 1/2+1/(6*a) 1/2-1/(6*a)];
end