function [x,v,a,t] = newmark(K,M,x0,v0,a,b,gamma,beta,udof,ndof,dt,N,Fd)
%********************************************************
% File: newmark.m
%   Performs linear Newmark time integration with proportional damping
% Syntax:
%   [x,v,a,t] = newmark(K,M,x0,v0,a,b,gamma,beta,udof,ndof,dt,N,Fd)
% Input:
%   K   : Global stiffness matrix
%   M   : Global mass matrix
%   x0  : Initial displacements
%   v0  : Initial velocities
%   a   : Weighting paramter for proportional damping
%   b   : Weighting paramter for proportional damping
%   gamma: Weighting parameter for Newmark 
%   beta: Weighting parameter for Newmark
%   udof: Unconstrained dof's
%   ndof: Numbor of dof's
%   dt  : Time-step size
%   N   : Number of time increments
%   Fd  : Dynamic loads
% Output:
%   x   : Nodal displacements
%   v   : Nodal velocities
%   a   : Nodal accelerations
%   t   : Discrete times
% Date:
%   Version 1.0    21.07.19
%********************************************************

% Rayleigh damping matrix
C  = a*M + b*K;

% Predicted mass
Ms = M(udof,udof) + gamma*dt*C(udof,udof) + beta*dt^2*K(udof,udof);

% Initial condition - acceleration
a0 = M(udof,udof)\(Fd(udof,1) - C(udof,udof)*v0(udof,1) - K(udof,udof)*x0(udof,1));

% Initialisation of response matrices
x = zeros(ndof,N+1); x = x(udof,:);
v = zeros(ndof,N+1); v = v(udof,:);
a = zeros(ndof,N+1); a = a(udof,:);

t = zeros(1,N+1);
x(:,1) = x0(udof);
v(:,1) = v0(udof);
a(:,1) = a0;

% Newmark time integration 
for i=1:N
    % Updated time increment 
    t(i+1) = t(i) + dt;
    
    % Prediction step
    v(:,i+1) = v(:,i) + (1 - gamma)*dt*a(:,i);
    x(:,i+1) = x(:,i) + dt*v(:,i) + (0.5 - beta)*dt^2*a(:,i);
    
    % Correction step
    a(:,i+1) = Ms\(Fd(udof,i+1) - C(udof,udof)*v(:,i+1) - K(udof,udof)*x(:,i+1));
    v(:,i+1) = v(:,i+1) + gamma*dt*a(:,i+1);
    x(:,i+1) = x(:,i+1) + beta*dt^2*a(:,i+1);
    
end