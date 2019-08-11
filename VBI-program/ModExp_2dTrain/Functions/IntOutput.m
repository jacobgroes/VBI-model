function [Fw,xxv,vvv,aav,xxh,vvh,aah] = IntOutput(b,bh,mw,cw,kw,l,a,v,...
    x,np,nt,g,i)
%**************************************************************************
% File: IntOutput.m
%   Calculates contact forces, vertical and horisontal bridge displacements
%   at contact points.
% Syntax:
%   [Fw,xxv,vvv,aav,xxh,vvh,aah] = IntOutput(b,bh,mw,cw,kw,l,a,v,...
%   x,np,nt,g,i)
% Input:
%   b   : Interaction matrix
%   bh  : Horisontal interaction matrix
%   mw  : Train wheel masses
%   cw  : Train suspension damping coefficients
%   kw  : Train suspension spring stiffness
%   l   : CG distance to front and rear wheel
%   a   : Nodal bridge accelerations
%   v   : Nodal bridge velocities
%   x   : Nodal bridge displacements
%   np  : Number of wheels
%   nt  : Number of trains
%   g   : Gravitational acceleration
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
for j=1:nt
        % Vehicle and bridge parameter definitions
        mf = mw(1,1,j); mr = mw(2,2,j); bf = b(:,1,j); br = b(:,2,j);
        cf = cw(1,1,j); cr = cw(2,2,j); kf = kw(1,1,j); kr = kw(2,2,j);
        
        ai = a(1:end-np,i+1); vi = v(1:end-np,i+1); xi = x(1:end-np,i+1);
        
        bhf = bh(:,1,j); bhr = bh(:,2,j); 
        
        lf = l(1,:,j);
        lr = l(2,:,j);
        
        % Velocities of train dof's
        vM = v(end-np+(1+(j-1)*2),i+1); 
        vThetaM = v(end-np+(2+(j-1)*2),i+1);
        
        % Displacements of train dof's
        uM = x(end-np+(1+(j-1)*2),i+1); 
        uThetaM = x(end-np+(2+(j-1)*2),i+1);
        
        % Iterator
        it = (j-1)*2;
        
        % Contact forces
        Fw(1+it,1) = -(mf*bf'*ai+cf*(bf'*vi-vM+lf*vThetaM)+kf*(bf'*xi-...
            uM+lf*uThetaM)+g*mf); % Front
        
        Fw(2+it,1) = -(mr*br'*ai+cr*(br'*vi-vM-lr*vThetaM)+kr*(br'*xi-...
            uM-lr*uThetaM)+g*mr); % Rear
        
        % Bridge response at interaction points
        xxv(1,1,j) = bf'*xi; xxv(2,1,j) = br'*xi; % Front; Rear;
        vvv(1,1,j) = bf'*vi; vvv(2,1,j) = br'*vi; % Front; Rear;
        aav(1,1,j) = bf'*ai; aav(2,1,j) = br'*ai; % Front; Rear;
        
        % Front
        xxh(1,1,j) = bhf'*xi/(l(2,1,j)/(sum(l(:,1,j)))); 
        vvh(1,1,j) = bhf'*vi/(l(2,1,j)/(sum(l(:,1,j))));  
        aah(1,1,j) = bhf'*ai/(l(2,1,j)/(sum(l(:,1,j))));  
        % Rear
        xxh(2,1,j) = bhr'*xi/(l(1,1,j)/(sum(l(:,1,j)))); 
        vvh(2,1,j) = bhr'*vi/(l(1,1,j)/(sum(l(:,1,j)))); 
        aah(2,1,j) = bhr'*ai/(l(1,1,j)/(sum(l(:,1,j)))); 
end