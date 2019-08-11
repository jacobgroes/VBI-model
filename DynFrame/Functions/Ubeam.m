function Uen = Ubeam(T,X,X3,U,nu)
%********************************************************
% File: Ubeam.m
%   Calculates displacements along the elements due to
%   nodal displacements in a group of spatial elastic 
%   beam elements.
% Syntax:
%   Uen = Ubeam(T,X,X3,U,nu)
% Input:
%   T  :  Topology array
%   X  :  Nodal coordinate array
%   X3 :  3rd node coordinate array
%   U  :  Global displacement array
%   nu :  Number of data points along the elements
% Output:
%  Uen :  Element displacement array
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Loop over elements
for e = 1:size(T,1)

    % Element arrays
    Xe  = X(T(e,1:2),:);
    Te  = T(e,:);
    X3e = X3(T(e,4),:);

    % Element displacement vector
    ue = uebeam(U,Te,Xe,X3e);

    % Element transformation matrix
    [Ae,He] = Aebeam(Xe,X3e);

    % 3-by-3 zero matrix
    O = zeros(3);

    % 6-by-6 transformation matrix
    An = [ He  O
            O He ];

    % Loop over discrete points
    for i = 1:nu

        % Normalised longitudinal coordinate
        s = (i-1)/(nu-1);

        % Interpolation matrix
        Nu = Nubeam(Xe,s);

        % Element deformation array
        Uen(e,i,:) = He'*Nu*ue;


    end
end
