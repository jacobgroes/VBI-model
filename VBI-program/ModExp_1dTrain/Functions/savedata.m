function [z] = savedata(np,xb,vb,ab,xtr,vtr,atr,Fw,t)
%**************************************************************************
% File: savedata.m
%   Defines additional parameters if desired, e.g. each suspension spring 
%   displacement. Saves the standard output as "output.mat".
% Syntax:
%   [z] = savedata(np,xb,vb,ab,xtr,vtr,atr,Fw,t)
% Input:
%   xb  : Nodal bridge displacements
%   vb  : Nodal bridge velocities
%   ab  : Nodal bridge accelerations
%   xtr : Train displacements
%   vtr : Train velocities
%   atr : Train acceleration
%   Fw  : Contact forces
%   t   : Time history
%   np  : Number of trains
% Output:
%   z   : Relative displacements of each suspension spring
% Date:
%   Version 1.0    20.06.19
%**************************************************************************
x0 = xtr(:,1);

% Vehicle response above wheel suspension
for j=1:np
z(j,:) = (xtr(j,:)-x0(j,1));
end

save('output','xb','vb','ab','xtr','vtr','atr','Fw','t')