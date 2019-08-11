function F = setdistload(F,X,T,X3,p)
%***************************************************
% File: setdistload.m
%   Transforms distributed element loads and sets
%   these into global vector form.
% Syntax:
%   F = setdistload(F,X,T,X3,p)
% Input:
%   F  : Initial global load vector
%   X  : System nodal coordinate array
%   T  : System topology array
%   X3 : 3rd node coordinate array
%   p  : Element loads given as
%        p = [ El px1 px2 py1 py2 pz1 pz2 ]
% Output:
%   F : Global load vector
% Date:
%   Version 1.0    27.07.11
%***************************************************

% Loop over distributed loaded elements
for e = 1:size(p,1)

  % Element arrays
  Xe  = X(T(p(e,1),1:2),:);
  Te  = T(p(e,1),:);
  X3e = X3(T(p(e,1),4),:);
  pe  = p(e,:)';
  
  % Element load vector
  Fe = Febeam(Xe,X3e,pe,4);
  
  % Assemble element load into global load
  F = assmF(F,Fe,Te);
  
end