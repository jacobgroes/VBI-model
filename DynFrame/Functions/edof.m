function ig = edof(Te)
%********************************************************
% File: edof.m
%   Defines the local dof adress vector (edof) of a given
%   type of finite element with topology row vector (Te)
%   number of element nodes(nn), local dof's per node(dof)
% Syntax:
%   edof = edof(Te)
% Input:
%   Te : Element topology array given as
%        Te = [ node 1, node 2, element prop no, (node3) ]
% Output:
%   ig : Adress vector for element dof's in relation to
%        the global dof's
% Date:
%   Version 1.0    17.02.11
%********************************************************

% Number of nodes for the element
nn = 2;

% Number of degrees-of-freedom for each node
dof = 6;

% Initialisation
ig  = zeros(1,nn*dof);

% Define global address vector for element dof's
for i = 1:nn
  for j = 1:dof
    ig((i-1)*dof+j) = (Te(i)-1)*dof + j;
  end
end