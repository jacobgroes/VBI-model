function Sen = Sbeam(T,X,G,X3,U,ns)
%********************************************************
% File: Sbeam.m
%   Calculates sectional forces along the elements due to
%   nodal displacements in a group of spatial elastic
%   beam elements.
% Syntax:
%   Sen = Sbeam(T,X,G,X3,U,ns)
% Input:
%   T  : System topology array
%   X  : System nodal coordinate array
%   G  : Material property array
%   X3 : 3rd node coordinate array
%   U  : System displacement vector
%   ns : Number of data points along the elements
% Output:
%   Sen : Element sectional force array
% Date:
%   Version 1.0    10.07.11
%********************************************************

% Loop over elements
for e = 1:size(T,1)
    
    % Element arrays
    Xe  = X(T(e,1:2),:);
    Te  = T(e,:);
    Ge  = G(T(e,3),:);
    X3e = X3(T(e,4),:);
    
    % Form initial element (column) vector
    a0 = (Xe(2,:)-Xe(1,:))';
    
    % Element length
    Le = sqrt(a0'*a0);
    
    % Element stiffness matrix
    [~,ke] = Kebeam(Xe,Ge,X3e,3);
    
    % Element displacement vector in local coordinates
    ue = uebeam(U,Te,Xe,X3e);
    
    % section force by local element stiffness matrix
    fe = ke*ue;
    
    % nodal forces at left node
    feA = fe(1:6); 
    
    % change of sign to section forces
    feA(1) = -feA(1);
    feA(4) = -feA(4);
    feA(6) = -feA(6);
        
    % nodal forces at right node
    feB = fe(7:12);
    
    % change of sign to section forces
    feB(2) = -feB(2);
    feB(3) = -feB(3);
    feB(5) = -feB(5);
    
    % Loop over sampling points
    for i = 1:ns
        
        % Normalised length coordinate
        s = (i-1)/(ns-1);
        
        % Linear interpolation of nodal values
        fen = feA*(1-s) + feB*s;
        
        % Re-ordering of section forces
        Sen(e,i,:) = fen([1 2 6 3 5 4]);
        
    end
end
