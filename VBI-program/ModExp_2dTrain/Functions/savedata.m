function [z] = savedata(nt,l,xb,vb,ab,xtr,vtr,atr,Fw,t)
%**************************************************************************
% File: savedata.m
%   Defines additional parameters if desired, e.g. each suspension spring 
%   displacement. Saves the standard output as "output.mat".
% Syntax:
%   [z] = savedata(np,xb,vb,ab,xtr,vtr,atr,Fw,t)
% Input:
%   nt  : Number of trains
%   l   : CG distances to front and rear wheel
%   xb  : Nodal bridge displacements
%   vb  : Nodal bridge velocities
%   ab  : Nodal bridge accelerations
%   xtr : Train displacements
%   vtr : Train velocities
%   atr : Train acceleration
%   Fw  : Contact forces
%   t   : Time history
% Output:
%   z   : Relative displacements of each suspension spring
% Date:
%   Version 1.0    20.06.19
%**************************************************************************
x0 = xtr(:,1);

% Vehicle response at wheel suspension [zf, zr]'
for j=1:nt
    zf(:,:,j) = (xtr(j*2-1,:)-x0((2*j-1),1))-(xtr(j*2,:)-...
        x0((2*j),1))*l(1,1,j);
    zr(:,:,j) = (xtr(j*2-1,:)-x0((2*j-1),1))+(xtr(j*2,:)-...
        x0((2*j),1))*l(2,1,j);
    
    z(:,:,j) = [zf(:,:,j)' zr(:,:,j)']';
end



save('output','xb','vb','ab','xtr','vtr','atr','Fw','t')