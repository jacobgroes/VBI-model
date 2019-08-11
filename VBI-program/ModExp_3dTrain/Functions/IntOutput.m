function [Fw,xxv,vvv,aav,xxh,vvh,aah] = IntOutput(b,bh,mw,cw,kw,...
                                                    l,lb,a,v,x,np,nt,g,i)
%**************************************************************************
% File: IntOutput.m
%   Calculates contact forces, vertical and horisontal bridge displacements
%   at contact points.
% Syntax:
%   [Fw,xxv,vvv,aav,xxh,vvh,aah] = IntOutput(b,bh,mw,cw,kw,l,lb,a,v,...
%   x,np,nt,g,i)
% Input:
%   b   : Interaction matrix
%   bh  : Horizontal interaction matrix
%   mw  : Train wheel masses
%   cw  : Train suspension damping coefficients
%   kw  : Train suspension spring stiffness
%   l   : CG distance to front and rear wheel
%   lb  : CG distance to left and right wheel
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
%   xxh : Horizontal bridge displacements at contact points
%   vvh : Horizontal bridge velocities at contact points
%   aah : Horizontal bridge accelerations at contact points
% Date:
%   Version 1.0    10.06.19
%**************************************************************************
for j=1:nt
        % Vehicle and bridge parameter definitions
        mf1 = mw(1,1,j); mr1 = mw(2,2,j); bf1 = b(:,1,j); br1 = b(:,2,j);
        cf1 = cw(1,1,j); cr1 = cw(2,2,j); kf1 = kw(1,1,j); kr1 = kw(2,2,j);
        
        mf2 = mw(3,3,j); mr2 = mw(4,4,j); bf2 = b(:,3,j); br2 = b(:,4,j);
        cf2 = cw(3,3,j); cr2 = cw(4,4,j); kf2 = kw(3,3,j); kr2 = kw(4,4,j);
        
        bhf1 = bh(:,1,j); bhr1 = bh(:,2,j); bhf2 = bh(:,3,j); bhr2 = bh(:,4,j);
        
        ai = a(1:end-np,i+1); vi = v(1:end-np,i+1); xi = x(1:end-np,i+1);
        
        ll = l(:,:,j);
        bb = lb(:,:,j);
        
        % Velocities of train dof's
        vM = v(end-np+(1+(j-1)*3),i+1); 
        vThetaY = v(end-np+(3+(j-1)*3),i+1);
        vThetaX = v(end-np+(2+(j-1)*3),i+1);
        
        % Displacements of train dof's
        uM = x(end-np+(1+(j-1)*3),i+1); 
        uThetaY = x(end-np+(3+(j-1)*3),i+1);
        uThetaX = x(end-np+(2+(j-1)*3),i+1);
        
        % Iterator
        it = (j-1)*4;
        
        % Contact forces
        Fw(1+it,1) = -(mf1*bf1'*ai+cf1*(bf1'*vi-vM+ll(1)*vThetaY+...
            bb(1)*vThetaX)+kf1*(bf1'*xi-uM+ll(1)*uThetaY+bb(1)*...
            uThetaX)+g*mf1); % Front 1
        
        Fw(2+it,1) = -(mr1*br1'*ai+cr1*(br1'*vi-vM-ll(2)*vThetaY+...
            bb(1)*vThetaX)+kr1*(br1'*xi-uM-ll(2)*uThetaY+bb(1)*...
            uThetaX)+g*mr1); % Rear 1
        
        Fw(3+it,1) = -(mf2*bf2'*ai+cf2*(bf2'*vi-vM+ll(1)*vThetaY-...
            bb(2)*vThetaX)+kf2*(bf2'*xi-uM+ll(1)*uThetaY-bb(2)*...
            uThetaX)+g*mf2); % Front 2
        
        Fw(4+it,1) = -(mr2*br2'*ai+cr2*(br2'*vi-vM-ll(2)*vThetaY-...
            bb(2)*vThetaX)+kr2*(br2'*xi-uM-ll(2)*uThetaY-bb(2)*...
            uThetaX)+g*mr2); % Rear 2
                
        % Vertical bridge response at interaction points
        xxv(1,1,j) = bf1'*xi; xxv(2,1,j) = br1'*xi; % Front; Rear;
        vvv(1,1,j) = bf1'*vi; vvv(2,1,j) = br1'*vi; % Front; Rear;
        aav(1,1,j) = bf1'*ai; aav(2,1,j) = br1'*ai; % Front; Rear;
        xxv(3,1,j) = bf2'*xi; xxv(4,1,j) = br2'*xi; % Front; Rear;
        vvv(3,1,j) = bf2'*vi; vvv(4,1,j) = br2'*vi; % Front; Rear;
        aav(3,1,j) = bf2'*ai; aav(4,1,j) = br2'*ai; % Front; Rear;
        
        % Horizontal bridge response at interaction points
        % Front left
        xxh(1,1,j) = bhf1'*xi/...
            (l(2,1,j)*lb(2,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        vvh(1,1,j) = bhf1'*vi/...
            (l(2,1,j)*lb(2,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        aah(1,1,j) = bhf1'*ai/...
            (l(2,1,j)*lb(2,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        % Rear left
        xxh(2,1,j) = bhr1'*xi/...
            (l(1,1,j)*lb(2,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));          
        vvh(2,1,j) = bhr1'*vi/...
            (l(1,1,j)*lb(2,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));         
        aah(2,1,j) = bhr1'*ai/...
            (l(1,1,j)*lb(2,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j)))); 
        % Front right
        xxh(3,1,j) = bhf2'*xi/...
            (l(2,1,j)*lb(1,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        vvh(3,1,j) = bhf2'*vi/...
            (l(2,1,j)*lb(1,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        aah(3,1,j) = bhf2'*ai/...
            (l(2,1,j)*lb(1,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        % Rear right
        xxh(4,1,j) = bhr2'*xi/... 
            (l(1,1,j)*lb(1,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        vvh(4,1,j) = bhr2'*vi/...
            (l(1,1,j)*lb(1,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
        aah(4,1,j) = bhr2'*ai/...
            (l(1,1,j)*lb(1,1,j)/(sum(l(:,1,j))*sum(lb(:,1,j))));
end