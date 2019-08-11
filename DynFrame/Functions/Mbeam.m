function M = Mbeam(M,T,X,G,X3)
%********************************************************
% File: Mbeam.m
%   Creates and assembles mass matrix for a group
%   of spatial elastic beam elements.
% Syntax:
%   M = Mbeam(M,T,X,G,X3)
% Input:
%   M  : Initial global mass matrix
%   T  : System topology array
%   X  : System nodal coordinate array
%   G  : Material property array
%   X3 : 3rd node coordinate array
% Output:
%   M : New global mass matrix
% Date:
%   Version 1.0    21.07.19
%********************************************************

% Loop over elements
for e = 1:size(T,1)
    
    % Element arrays
    Xe  = X(T(e,1:2),:);
    Te  = T(e,:);
    Ge  = G(T(e,3),:);
    X3e = X3(T(e,4),:);
    
    % Element mass matrix
    [Me,~] = Mebeam(Xe,Ge,X3e,4);
    
    % Assemble mass matrix
    M = assmK(M,Me,Te);
end

% Remove potential asymmetry from numerical integration
M = 1/2*M + 1/2*M';