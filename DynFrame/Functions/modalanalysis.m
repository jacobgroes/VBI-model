function [S,x,v,a,t,mm,km,f,r0,s0,omega] = modalanalysis(K,M,zeta,x0,...
    v0,udof,Fd,N,dt)
%********************************************************
% File: modalanalysis.m
%   Solves the eigenvalue problem and performs modal analysis by the
%   time-stepping algorithm timestep2.m
% Syntax:
%   [S,x,v,a,t,mm,km,f,r0,s0,omega] = modalanalysis(K,M,zeta,x0,...
%                                     v0,udof,Fd,N,dt)
% Input:
%   K   : Global stiffness matrix
%   M   : Global mass matrix
%   zeta: Modal damping ratio
%   x0  : Initial displacements
%   v0  : Initial velocities
%   udof: Unconstrained dof's
%   Fd  : Dynamic loads
%   N   : Number of time increments
%   dt  : Time-step size
% Output:
%   S   : Matrix of mode-shapes
%   x   : Nodal displacements
%   v   : Nodal velocities
%   a   : Nodal accelerations
%   t   : Discrete times
%   km  : Modal stiffness matrix
%   mm  : Modal mass matrix
%   f   : Modal loads
%   r0  : Modal initial displacements
%   s0  : Modal initial velocities
%   omega: Natural angular frequencies
% Date:
%   Version 1.0    21.07.19
%********************************************************

% Number of free dof's
n = sum(udof);

% Generalized eigenvalue problem 
[S,val] = eig(K(udof,udof),M(udof,udof));
omega = sqrt(diag(val));

% modal mass and stiffness
mm = S'*M(udof,udof)*S;
km = S'*K(udof,udof)*S;

% initial conditions
r0 = mm\S'*M(udof,udof)*x0(udof);
s0 = mm\S'*M(udof,udof)*v0(udof);

% modal load
f = mm\(S'*Fd(udof,:));

% damping matrix
zeta = zeta*ones(n,1);

% modal time histories
r = zeros(n,N+1);
s = zeros(n,N+1);
d = zeros(n,N+1);

for  j = 1:n    
    % time response of mode j
    [rj,sj,dj,t] = timestep2(omega(j),zeta(j),r0(j),s0(j),0,dt,N,f(j,:));
    
    % store response in modal vector
    r(j,:) = rj;
    s(j,:) = sj;    
    d(j,:) = dj;
end

% response history by superposition
x = S*r;
v = S*s;
a = S*d;