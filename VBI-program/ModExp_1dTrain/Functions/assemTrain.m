function [mt,mb,kw,cw] = assemTrain(mt,mb,kw,cw,np)
%**************************************************************************
% File: assemTrain.m
%   Assembles the modal system matrices governing the train and arranges
%   the wheel masses in diagonal matrix.
% Syntax:
%   [mt,mb,kw,cw] = assemTrain(mt,mb,kw,cw,np)
% Input:
%   mt  : Train rigid mody mass properties
%   mb  : Wheel masses
%   kw  : Suspension spring stiffness
%   cw  : Suspension damping coefficient
%   np  : Number of trains
% Output:
%   mt  : Train mass matrix
%   mb : Wheel mass matrix
%   kw  : Train stiffness matrix
%   cw  : Train damping matrix

% Date:
%   Version 1.0    10.06.19
%**************************************************************************
% Train mass matrix
mt = diag(mt);
% Wheel masses
mb = diag(mb);
% Train stiffness matrix
kw = diag(kw);
% Train damping matrix
cw = diag(cw);
