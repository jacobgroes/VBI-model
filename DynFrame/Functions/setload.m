function F = setload(F,P,dof)
%***************************************************
% File: setload.m
%   Sets the nodal loads in global vector form.
% Syntax:
%   F = setload(P,F)
% Input:
%   F : Initial global load vector
%   P : Nodal load array given as 
%       P = [Nodeno, dof, load value]
% Output:
%   F : Global load vector
% Date:
%   Version 1.0    10.02.11
%***************************************************

% Loop over loaded dof
for i = 1:size(P,1)
    
    % Put loads into global load vector
    F(dof*P(i,1)-dof+P(i,2)) = F(dof*P(i,1)-dof+P(i,2)) + P(i,3);

end