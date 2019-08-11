function [M,K,C] = CSM(b,bh,mm,km,cm,mt,mb,kw,cw,O)
%**************************************************************************
% File: CSM.m
%   Assembles the coupled system matrices M, K and C.
% Syntax:
%   [M,K,C] = CSM(b,bh,mm,km,cm,mt,mb,kw,cw,O)
% Input:
%   b   : Modal interaction matrix
%   bh  : Horizontal modal interaction matrix 
%   mm  : Bridge modal mass matrix
%   km  : Bridge modal stiffness matrix
%   cm  : Bridge modal damping matrix
%   mt  : Train mass matrix
%   mb  : Train wheel masses
%   kw  : Train stiffness matrix
%   cw  : Train damping matrix
%   O   : n x n matrix of zeros
% Output:
%   M   : Coupled mass matrix
%   K   : Coupled stiffness matrix
%   C   : Coupled damping matrix
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
% Coupled mass matrix
M = [mm+b*mb*b'+bh*mt*bh'    O
      O'          mt];

% Coupled stiffness matrix
K =  [km+b*kw*b' -b*kw
     -kw*b'       kw];

% Coupled damping matrix
C =  [cm+b*cw*b' -b*cw
     -cw*b'       cw];