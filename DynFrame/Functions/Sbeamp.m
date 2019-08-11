function Sen = Sbeamp(T,X,X3,p,ns)
%********************************************************
% File: Sbeamp.m
%   Calculates sectional forces along the elements due
%   to distributed loads in a group of spatial elastic
%   beam elements.
% Syntax:
%   Sen = Sbeamp(T,X,X3,p,ns)
% Input:
%   T  :  Topology matrix
%   X  :  Nodal coordinate matrix
%   X3 :  3rd node coordinates
%   p  :  Element distributed loads
%         p = [ el px1 px2 py1 py2 pz1 pz2 ]
%   nu :  Number of data points along the elements
% Output:
%  Uen :  Element displacement array
% Date:
%   Version 1.0    26.07.11
%********************************************************

% Initialisation
Sen = zeros(size(T,1),ns,6);

% Loop over loaded elements
for e = 1:size(p,1)

    % Element arrays
    Xe  = X(T(p(e,1),1:2),:);
    X3e = X3(T(p(e,1),4),:);
    
    % Element end load values
    p1x = p(e,2);   
    p2x = p(e,3);   
    p1y = p(e,4);
    p2y = p(e,5);
    p1z = p(e,6);
    p2z = p(e,7);

    % Form initial element (column) vector
    a0 = (Xe(2,:)-Xe(1,:))';

    % Element length
    Le = sqrt(a0'*a0);

    % Loop over data points
    for i = 1:ns

        % Normalised longitudinal coordinate
        s = (i-1)/(ns-1);

        % Axial force
        Sen(p(e,1),i,1) = Le * ( 3*(p1x-p2x)*s^2 - 6*p1x*s + (p2x+2*p1x) ) / 6;

        % Shear force (in y-direction)
        Sen(p(e,1),i,2) = Le * ( 60*(p2y-p1y)*s^2 + 120*p1y*s - (18*p2y+42*p1y) ) /120;
        
        % Bending moment (about z-axis)
        Sen(p(e,1),i,3) = Le^2 * ( 20*(p2y-p1y)*s^3 + 60*p1y*s^2 - (18*p2y+42*p1y)*s + (4*p2y+6*p1y) ) / 120;

        % Shear force (in z-direction)
        Sen(p(e,1),i,4) = Le * ( 60*(p2z-p1z)*s^2 + 120*p1z*s - (18*p2z+42*p1z) ) /120;

        % Bending moment (about y-axis)
        Sen(p(e,1),i,5) = Le^2 * ( 20*(p2z-p1z)*s^3 + 60*p1z*s^2 - (18*p2z+42*p1z)*s + (4*p2z+6*p1z) ) / 120;
      
    end
end
