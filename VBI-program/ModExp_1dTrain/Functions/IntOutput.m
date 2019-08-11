function [Fw,xxv,vvv,aav,xxh,vvh,aah] = IntOutput(mw,ct,kt,g,b,bh,x,v,...
    a,np,i)
%**************************************************************************
% File: IntOutput.m
%   Calculates contact forces, vertical and horisontal bridge displacements
%   at contact points.
% Syntax:
%   [Fw,xxv,vvv,aav,xxh,vvh,aah] = IntOutput(b,bh,mw,cw,kw,l,a,v,...
%   x,np,nt,g,i)
% Input:
%   mw  : Train wheel masses
%   ct  : Train suspension damping coefficients
%   kt  : Train suspension spring stiffness
%   g   : Gravitational acceleration
%   b   : Interaction matrix
%   bh  : Horisontal interaction matrix
%   l   : CG distance to front and rear wheel
%   x   : Nodal bridge displacements
%   a   : Nodal bridge accelerations
%   v   : Nodal bridge velocities
%   np  : Number of contact points
%   i   : Current time increment
% Output:
%   Fw  : Contact forces
%   xxv : Vertical bridge displacements at contact points
%   vvv : Vertical bridge velocities at contact points 
%   aav : Vertical bridge accelerations at contact points
%   xxh : Horisontal bridge displacements at contact points
%   vvh : Horisontal bridge velocities at contact points
%   aah : Horisontal bridge accelerations at contact points
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
for j=1:np
        % Contact forces
        Fw(j,1) = -(mw(j,j)*b(:,j)'*a(1:end-np,i+1)+ct(j,j)*(b(:,j)'*...
            v(1:end-np,i+1)-v(end-np+j,i+1))+kt(j,j)*(b(:,j)'*...
            x(1:end-np,i+1)-x(end-np+j,i+1))+g(1)*mw(j,j));
        
        % Bridge response at interaction points
        xxv(j,1) = b(:,j)'*x(1:end-np,i+1); xxh(j,1) = bh(:,j)'*...
            x(1:end-np,i+1); 
        
        vvv(j,1) = b(:,j)'*v(1:end-np,i+1); vvh(j,1) = bh(:,j)'*...
            v(1:end-np,i+1); 
        
        aav(j,1) = b(:,j)'*a(1:end-np,i+1); aah(j,1) = bh(:,j)'*...
            a(1:end-np,i+1);
        
end