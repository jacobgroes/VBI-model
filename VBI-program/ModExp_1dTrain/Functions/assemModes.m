function [S] = assemModes(u,y,n)
%**************************************************************************
% File: assemModes.m
%   Rearranges mode-shape input
% Syntax:
%   [S] = assemModes(u,y,n)
% Input:
%   u   : Mode-shape input
%   y   : Unit prefix
%   n   : Number of modes included in the analysis
% Output:
%   S   : Matrix of mode-shapes
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
S = zeros(length(u(:,1,1))*6,n);
S(1:6:end,:) = u(:,1,1:n)/(10^y);
S(2:6:end,:) = u(:,2,1:n)/(10^y);
S(3:6:end,:) = u(:,3,1:n)/(10^y);
S(4:6:end,:) = u(:,4,1:n)/(10^y);
S(5:6:end,:) = u(:,5,1:n)/(10^y);
S(6:6:end,:) = u(:,6,1:n)/(10^y); 