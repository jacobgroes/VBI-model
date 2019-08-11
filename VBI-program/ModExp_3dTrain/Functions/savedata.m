function [z] = savedata(nt,ll,lb,xb,vb,ab,xtr,vtr,atr,Fw,t)
%**************************************************************************
% File: savedata.m
%   Defines additional parameters if desired, e.g. each suspension spring 
%   displacement. Saves the standard output as "output.mat".
% Syntax:
%   [z] = savedata(nt,ll,lb,xb,vb,ab,xtr,vtr,atr,Fw,t)
% Input:
%   nt  : Number of trains
%   ll  : CG distances to front and rear wheel
%   lb  : CG distances to left and right wheel
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
zf1(:,:,j) = (xtr(j*3-2,:)-x0((3*j-2),1))-(xtr(j*3,:)-...
    x0((3*j),1))*ll(1,1,j)-(xtr(j*3-1,:)-x0((3*j-1),1))*lb(1,1,j);

zr1(:,:,j) = (xtr(j*3-2,:)-x0((3*j-2),1))+(xtr(j*3,:)-...
    x0((3*j),1))*ll(2,1,j)-(xtr(j*3-1,:)-x0((3*j-1),1))*lb(1,1,j);

zf2(:,:,j) = (xtr(j*3-2,:)-x0((3*j-2),1))-(xtr(j*3,:)-...
    x0((3*j),1))*ll(1,1,j)+(xtr(j*3-1,:)-x0((3*j-1),1))*lb(2,1,j);

zr2(:,:,j) = (xtr(j*3-2,:)-x0((3*j-2),1))+(xtr(j*3,:)-...
    x0((3*j),1))*ll(2,1,j)+(xtr(j*3-1,:)-x0((3*j-1),1))*lb(2,1,j);

z(:,:,j) = [zf1(:,:,j)' zr1(:,:,j)' zf2(:,:,j)' zr2(:,:,j)']';
end

save('output','xb','vb','ab','xtr','vtr','atr','Fw','t')