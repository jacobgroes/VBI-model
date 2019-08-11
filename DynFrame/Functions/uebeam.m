function ue = uebeam(U,Te,Xe,X3e)
%********************************************************
% File: uebeam.m
%   Creates the element displacement vector of a spatial
%   elastic beam element.
% Syntax:
%   ue = uebeam(U,Te,Xe,X3e)
% Input:
%    U : Global displacement vector
%   Te : Element topology array given as
%        Te = [node 1, node 2, element prop no, node3 ]

% Output:
%   ue : Element displacement vector
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Adress vector for element dof's
ig = edof(Te);

% Element displacement vector in global coordinates
Ue = U(ig);

% Element transformation matrix
Ae = Aebeam(Xe,X3e);

% Element displacement vector local coordinates
ue = Ae*Ue;

