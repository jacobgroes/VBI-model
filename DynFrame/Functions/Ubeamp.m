function Uen = Ubeamp(T,X,G,X3,p,nu)
%********************************************************
% File: Ubeamp.m
%   Calculates displacements along the elements due to
%   distributed loads in a group of spatial elastic
%   beam elements.
% Syntax:
%   Uen = Ubeamp(T,X,G,X3,p,nu)
% Input:
%   T  : Topology matrix
%   X  : Nodal coordinate matrix
%   G  : Element constitutive array
%   X3 : 3rd node coordinates
%   p  : Element distributed loads,
%        p = [ el px1 px2 py1 py2 pz1 pz2 ]
%   nu : Number of data points along the elements
% Output:
%  Uen : Element displacement array
% Date:
%   Version 1.0    27.07.11
%********************************************************

% Initialisation
Uen = zeros(size(T,1),nu,3);

% Loop over loaded elements
for e = 1:size(p,1)

    % Initialisation
    Ue = zeros(1,3);

    % Element arrays
    Xe  = X(T(p(e,1),1:2),:);
    Ge  = G(T(p(e,1),3),:);
    X3e = X3(T(p(e,1),4),:);
    
    % Element end load values
    p1x = p(e,2);   p1y = p(e,4);   p1z = p(e,6);
    p2x = p(e,3);   p2y = p(e,5);   p2z = p(e,7);

    % Form initial element (column) vector
    a0 = (Xe(2,:)-Xe(1,:))';

    % Element length
    Le = sqrt(a0'*a0);

    % Element transformation matrix
    [Ae,He] = Aebeam(Xe,X3e);

    % Element properties
    EA = Ge(1);  EIy = Ge(2);  EIz = Ge(3);

    % Loop over data points
    for i = 1:nu

        % Normalised length coordinate
        s = (i-1)/(nu-1);

        % Axial displacement
        Ue(1) = Le^2 * ( (p1x-p2x)*s^3-3*p1x*s^2+(p2x+2*p1x)*s ) / (6*EA);
        % Transverse displacement (local y-direction)
        Ue(2) = Le^4 * ( (p2y-p1y)*s^5+5*p1y*s^4-(3*p2y+7*p1y)*s^3+(2*p2y+3*p1y)*s^2 ) / (120*EIy);
        % Transverse displacement (local z-direction)
        Ue(3) = Le^4 * ( (p2z-p1z)*s^5+5*p1z*s^4-(3*p2z+7*p1z)*s^3+(2*p2z+3*p1z)*s^2 ) / (120*EIz);
        % Transform and add element displ. constribution
        Uen(p(e,1),i,:) = He'*Ue';
    end
end
