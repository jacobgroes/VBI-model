function [xb,vb,ab,xxv,xxh,Fw,xtr,vtr,atr] = MainProgram(n,nno,X,X3,T,...
    gamma,beta,dt,N,tt,initial,np,L,vt,e,eh,Mt,mw,Kt,Ct,mm,km,cm,g,Fi,S)
%**************************************************************************
% File: MainProgram.m
%   Main script for calculation of the linear-elastic dynamic response of 
%   bridge and vehicles. Includes calculation of the interaction matrix, 
%   calculation and update of system matrices in each time step and 
%   response calculation by the Newmark Time Integration Method.
% Syntax:
%   [xb,vb,ab,xxv,xxh,Fw,xtr,vtr,atr] = MainProgram(n,nno,X,X3,T,...
%   gamma,beta,dt,N,tt,initial,np,L,vt,e,eh,Mt,mw,Kt,Ct,mm,km,cm,g,Fi,S)
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
%   np  : Number of trains
%   L   : Bridge length
%   vt  : Constant train velocity
%   e   : horizontal train CM eccentricity to bridge shear center
%   eh  : Vertical train CM eccentricity to bridge shear center
%   Mt  : Train mass matrix
%   mw  : Train wheel masses
%   Kt  : Train stiffness matrix
%   Ct  : Train damping matrix
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

% External bridge loads in modal coordinates
Fi = S'*Fi;

% Gravitational acceleration vector 
g = ones(np,1)*g;

% element lengths
le = sqrt((X(T(:,2),1)-X(T(:,1),1)).^2+(X(T(:,2),2)-X(T(:,1),2)).^2+...
    (X(T(:,2),3)-X(T(:,1),3)).^2);

% accumulated sum of element lengths
la = zeros(length(le)+1,1);
for j=1:length(le)
    la(j+1) = le(j) + la(j);
end

% Initialization of interaction matrix
bin = zeros(nno*6,np);

% Wheel positions and index of interacting wheels
[pos,intw] = IntPos(bin,initial,vt,tt,0,L);

% Modal vertical interaction matrix at time increment i=1
b = IntMat(pos,X,X3,T,bin,e,intw,le,la,S);

% Modal horizontal "interaction" matrix at time increment i=1
bh = IntMatH(pos,X,X3,T,eh,bin,intw,le,la,S);

% Matrix of zeros
O = zeros(n,np);

% Initialization of system matrices
[M,K,C] = CSM(b,bh,mm,km,cm,Mt,mw,Kt,Ct,O);

% Initial load vector
F = [Fi(:,1)-b*mw*g; -Mt*g];
              
% initial conditions
x0 = zeros(n+np,1);
v0 = zeros(n+np,1);
x0(end+1-np:end,1) = Kt\F(end+1-np:end,1); 

% Initial condition - acceleration
a0 = M\(F(:,1) - C*v0(:,1) - K*x0(:,1));

% Response matrices
x   = zeros(n+np,N+1);
v   = zeros(n+np,N+1);
a   = zeros(n+np,N+1);
t   = zeros(1,N+1);

% Initial conditions
x(:,1) = x0;
v(:,1) = v0;
a(:,1) = a0;

% Contact forces and vertical and horizontal bridge response at
% wheel-bridge interaction points
[Fw(:,1),xxv(:,1),vvv(:,1),aav(:,1),xxh(:,1),vvh(:,1),aah(:,1)]...
    = IntOutput(mw,Ct,Kt,g,b,bh,x,v,a,np,0);

% Newmark time integration with time varying system parameters
for i=1:N
    % Updating time increment 
    t(i+1) = t(i) + dt;
    
    % Wheel positions and index of interacting wheels
    [pos,intw] = IntPos(bin,initial,vt,tt,i,L);  
    
    % Modal vertical interaction matrix 
    b = IntMat(pos,X,X3,T,bin,e,intw,le,la,S);
    
    % Modal horizontal "interaction" matrix 
    bh = IntMatH(pos,X,X3,T,eh,bin,intw,le,la,S);
    
    % Updated system matrices
    [M,K,C] = CSM(b,bh,mm,km,cm,Mt,mw,Kt,Ct,O);
    
    % Coupled loads
    F(:,i+1) = [Fi(:,i+1)-b*mw*g; -Mt*g];
                            
    % Prediction step
    v(:,i+1) = v(:,i) + (1 - gamma)*dt*a(:,i);
    x(:,i+1) = x(:,i) + dt*v(:,i) + (0.5 - beta)*dt^2*a(:,i);     
               
    % Predicted mass
    Ms = M + gamma*dt*C + beta*dt^2*K;
                         
    % Correction step 
    a(:,i+1) = Ms\(F(:,i+1) - C*v(:,i+1) - K*x(:,i+1));
    v(:,i+1) = v(:,i+1) + gamma*dt*a(:,i+1);
    x(:,i+1) = x(:,i+1) + beta*dt^2*a(:,i+1);
    
    % Contact forces and vertical and horizontal bridge response at
    % wheel-bridge interaction points
    [Fw(:,i+1),xxv(:,i+1),vvv(:,i+1),aav(:,i+1),xxh(:,i+1),...
        vvh(:,i+1),aah(:,i+1)] = IntOutput(mw,Ct,Kt,g,b,bh,x,v,...
        a,np,i);

end

% Bridge response in generalised coordinates
xb = S*x(1:end-np,:);
vb = S*v(1:end-np,:);
ab = S*a(1:end-np,:);

% Vehicle response in generalised coordinates
xtr = x(end-(np-1):end,:);
vtr = v(end-(np-1):end,:);
atr = a(end-(np-1):end,:);
