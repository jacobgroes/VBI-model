function K = Kbeam(K,T,X,G,X3)
%********************************************************
% File: Kbeam.m
%   Creates and assembles stiffness matrix for a group
%   of spatial elastic beam elements.
% Syntax:
%   K = Kbeam(K,T,X,G,X3)
% Input:
%   K  : Initial global stiffness matrix
%   T  : System topology array
%   X  : System nodal coordinate array
%   G  : Material property array
%   X3 : 3rd node coordinate array
% Output:
%   K : New global stiffness matrix
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Loop over elements
for e = 1:size(T,1)
    
    % Element arrays
    Xe  = X(T(e,1:2),:);
    Te  = T(e,:);
    Ge  = G(T(e,3),:);
    X3e = X3(T(e,4),:);
    
    % Element stiffness matrix
    [Ke,~] = Kebeam(Xe,Ge,X3e,4);
    
    % Assemble stiffness matrix
    K = assmK(K,Ke,Te);
end

% Remove potential asymmetry from numerical integration
K = 1/2*K + 1/2*K';
