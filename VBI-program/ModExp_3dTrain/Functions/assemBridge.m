function [mm,km,cm] = assemBridge(m,n,omega,zeta)
%**************************************************************************
% File: assemBridge.m
%   Assembles the modal bridge matrices mm, km and cm.
% Syntax:
%   [mm,km,cm] = assemBridge(m,n,omega,zeta)
% Input:
%   m   : Modal mass
%   n   : Number of modes included in the analysis
%   omega: Natural angular frequencies
%   zeta: Modal damping ratio
% Output:
%   mm  : Modal mass matrix
%   km  : Modal stiffness matrix
%   cm  : Modal damping matrix
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
% Modal mass matrix
mm = m*ones(n,1); 
mm = diag(mm);

% Modal stiffness matrix
km = mm*omega.^2;
km = diag(km);

% Viscous damping coefficent matrix
cm = 2*zeta*sqrt(km*mm);