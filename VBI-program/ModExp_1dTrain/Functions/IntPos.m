function [pos,intw] = IntPos(bin,initial,vt,tt,i,L)
%**************************************************************************
% File: IntPos.m
%   Identifies the wheels in contact with the bridge and defines their 
%   positions. 
% Syntax:
%   b = IntPos(bin,initial,vt,tt,i,L)
% Input:
%   bin : Initialised interaction matrix
%   Initial: Initial positions of all wheels
%   vt  : Constant train velocity
%   tt  : Time increment vector
%   i   : Current time-step
%   L   : Bridge length
% Output:
%   pos : Wheel positions
%   intw: Index of interacting wheels
% Date:
%   Version 1.0    10.07.19
%**************************************************************************
    % Load positions
    pos=initial + vt*tt(i+1);
    
    % Exclusion of off-bridge locations (pos<0 || pos>L)
    logic1 = pos>=0; % Location of first bridge node
    logic2 = pos<=L; % Bridge length
    logics = logic1+logic2;
    
    % If both criteria are met, logics == 2
    logic = logics==2;
    
    % Bridge-vehicle contact positions at i'th time increment
    pos = pos(logic); 
    
    % Train wheels currently interacting with the beam at contact positions
    intw = 1:length(bin(1,:));
    intw = intw(logic);
    

