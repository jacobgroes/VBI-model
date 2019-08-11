function [xb,vb,ab,xxv,xxh,Fw,xtr,vtr,atr] = MainProgram(n,nno,X,X3,T,...
    gamma,beta,dt,N,tt,initial,nt,L,l,vt,e,eh,Mt,mw,Kt,kw,Ct,cw,mm,km,...
    cm,g,Fi,S)
%**************************************************************************
% File: MainProgram.m
%   Main script for calculation of the linear-elastic dynamic response of 
%   bridge and vehicles. Includes calculation of the interaction matrix, 
%   calculation and update of system matrices in each time step and 
%   response calculation by the Newmark Time Integration Method.
% Syntax:
%   [xb,vb,ab,xxv,xxh,Fw,xtr,vtr,atr] = MainProgram(n,nno,X,X3,T,...
%   gamma,beta,dt,N,tt,initial,nt,L,l,vt,e,eh,Mt,mw,Kt,kw,Ct,cw,mm,km,...
%   cm,g,Fi,S)
% Input:
%   n   : Number of modes included in analysis
%   nno : Number of nodes in FE-model
%   X   : Node coordinate array
%   X3  : Third node coordinates
%   T   : Topology array
%   gamma: Newmark time integration weighting parameter
%   beta: Newmark time integration weighting parameter
%   dt  : Time increment size
%   N   : Number of time increments
%   tt  : Time increment vector
%   initial: Initial positions of all wheels
%   nt  : Number of trains
%   L   : Bridge length
%   l   : Distance from train CG to front and rear wheel
%   vt  : Constant train velocity
%   e   : Train CG eccentricity to bridge shear center
%   eh  : Vertical train CM eccentricity to bridge shear center
%   Mt  : Train mass matrix
%   mw  : Train wheel masses
%   Kt  : Train stiffness matrix
%   kw  : Train suspension spring stiffness
%   Ct  : Train damping matrix
%   cw  : Train suspension damping coefficient
%   mm  : Bridge modal mass matrix
%   km  : Bridge modal stiffness matrix
%   cm  : Bridge modal damping matrix
%   g   : Gravitational acceleration
%   Fi  : Initial load vector
%   S   : Mass normalised mode-shapes
% Output:
%   xb  : Bridge displacement history in geometric coordinates 
%   vb  : Bridge velocity history in geometric coordinates 
%   ab  : Bridge acceleration history in geometric coordinates 
%   xxv : Vertical bridge displacement at train interaction points (plot)
%   xxh : horizontal bridge displacement at train interaction points (plot)
%   Fw  : Wheel-bridge contact force history
%   xtr : Train displacement history
%   vtr : Train velocity history
%   atr : Train acceleration history
% Date:
%   Version 1.0    10.07.19
%**************************************************************************

% Number of train dof's 
np = nt*2;

% External bridge loads in modal coordinates
Fi = S'*Fi;

% Element lengths
le = sqrt((X(T(:,2),1)-X(T(:,1),1)).^2+(X(T(:,2),2)-X(T(:,1),2)).^2+...
    (X(T(:,2),3)-X(T(:,1),3)).^2);

% Accumulated sum of element lengths
la = zeros(length(le)+1,1);
for j=1:length(le)
    la(j+1) = le(j) + la(j);
end

% Initialization of interaction matrix
bin = zeros(nno*6,2,nt);

% Wheel positions and index of interacting wheels
[pos,intw] = IntPos(bin,initial,vt,tt,0,L);

% Modal vertical interaction matrix at time increment i=1
b = IntMat(pos,X,X3,T,bin,e,intw,le,la,S,nt);

% Modal horizontal "interaction" matrix at time increment i=1
bh = IntMatH(pos,X,X3,T,eh,bin,intw,le,la,S,nt,l);

% Initialization of system matrices
[M,K,C] = CSM(b,bh,mm,km,cm,Mt,mw,kw,cw,Kt,Ct,l,nt,n);

% Response matrices
x   = zeros(n+2*nt,N+1);
v   = zeros(n+2*nt,N+1);
a   = zeros(n+2*nt,N+1);
t   = zeros(1,N+1);

% initial conditions
x0 = zeros(n+2*nt,1);
v0 = zeros(n+2*nt,1);

% Initial load vector at time increment i=1
F = assemLoad(Fi,b,mw,g,Mt,nt,n,1);

% Initial gravitational train displacements
x0 = gravDisp(F,nt,n,Kt,x0);

% Initial condition - acceleration
a0 = M\(F - C*v0 - K*x0);

% Initial conditions
x(:,1) = x0(:,1);
v(:,1) = v0(:,1);
a(:,1) = a0(:,1);

% Contact forces and vertical and horizontal bridge response at
% wheel-bridge interaction points
[Fw(:,1,:),xxv(:,1,:),vvv(:,1,:),aav(:,1,:),xxh(:,1,:),vvh(:,1,:),...
    aah(:,1,:)] = IntOutput(b,bh,mw,cw,kw,l,a,v,x,np,nt,g,0);

% Newmark time integration with time varying system parameters
for i=1:N
    % Updating time increment 
    t(i+1) = t(i) + dt;
    
    % Wheel positions and index of interacting wheels
    [pos,intw] = IntPos(bin,initial,vt,tt,i,L);
    
    % Modal vertical interaction matrix 
    b = IntMat(pos,X,X3,T,bin,e,intw,le,la,S,nt);
    
    % Modal horizontal "interaction" matrix 
    bh = IntMatH(pos,X,X3,T,eh,bin,intw,le,la,S,nt,l);
    
    % Updated system matrices
    [M,K,C] = CSM(b,bh,mm,km,cm,Mt,mw,kw,cw,Kt,Ct,l,nt,n);
    
    % Train load
    F(:,i+1) = assemLoad(Fi,b,mw,g,Mt,nt,n,i+1);
    
    % Predicted mass
    Ms = M + gamma*dt*C + beta*dt^2*K;
    
    % Prediction step
    v(:,i+1) = v(:,i) + (1 - gamma)*dt*a(:,i);
    x(:,i+1) = x(:,i) + dt*v(:,i) + (0.5 - beta)*dt^2*a(:,i);

    % Correction step
    a(:,i+1) = Ms\(F(:,i+1) - C*v(:,i+1) - K*x(:,i+1));
    v(:,i+1) = v(:,i+1) + gamma*dt*a(:,i+1);
    x(:,i+1) = x(:,i+1) + beta*dt^2*a(:,i+1);
    
    % Contact forces and vertical and horizontal bridge response at
    % wheel-bridge interaction points
    [Fw(:,i+1,:),xxv(:,i+1,:),vvv(:,i+1,:),aav(:,i+1,:),xxh(:,i+1,:),...
        vvh(:,i+1,:),aah(:,i+1,:)] = IntOutput(b,bh,mw,cw,kw,l,a,v,x,...
        np,nt,g,i);
    
end

% Bridge response in geometric coordinates 
xb = S*x(1:end-2*nt,:);
vb = S*v(1:end-2*nt,:);
ab = S*a(1:end-2*nt,:);

% Vehicle response in geometric coordinates
xtr = x(end-(2*nt-1):end,:);
vtr = v(end-(2*nt-1):end,:);
atr = a(end-(2*nt-1):end,:);
