function [x,v,a,t] = Mnewmark(km,mm,f,zeta,r0,s0,N,dt,gamma,beta,S)
%********************************************************
% File: Mnewmark.m
%   Performs linear Newmark time integration in modal coordinates
% Syntax:
%   [x,v,a,t] = Mnewmark(km,mm,f,zeta,r0,s0,N,dt,gamma,beta,S)
% Input:
%   km  : Modal stiffness matrix
%   mm  : Modal mass matrix
%   f   : Modal loads
%   zeta: Modal damping ratio
%   r0  : Modal initial displacements
%   s0  : Modal initial velocities
%   dt  : Time-step size
%   N   : Number of time increments
%   gamma: Weighting parameter for Newmark 
%   beta: Weighting parameter for Newmark
%   udof: Unconstrained dof's
%   ndof: Numbor of dof's
%   S   : Matrix of mode-shapes
% Output:
%   x   : Nodal displacements
%   v   : Nodal velocities
%   a   : Nodal accelerations
%   t   : Discrete times
% Date:
%   Version 1.0    21.07.19
%********************************************************


% Number of modes
n = length(mm(1,:));

% Diagonal modal damping matrix
cm = diag(2*zeta*sqrt(diag(km).*diag(mm)));

% Predicted mass
Ms = mm + gamma*dt*cm + beta*dt^2*km;

% Initial condition - acceleration
d0 = mm\(f(:,1) - cm*s0 - km*r0);

% Initialisation of response matrices
x = zeros(n,N+1); 
v = zeros(n,N+1); 
a = zeros(n,N+1); 
  
t = zeros(1,N+1);
x(:,1) = r0;
v(:,1) = s0;
a(:,1) = d0;

% Newmark time integration scheme
for i=1:N
    % Updated time increment 
    t(i+1) = t(i) + dt;
    
    % Prediction step
    v(:,i+1) = v(:,i) + (1 - gamma)*dt*a(:,i);
    x(:,i+1) = x(:,i) + dt*v(:,i) + (0.5 - beta)*dt^2*a(:,i);
    
    % Correction step
    a(:,i+1) = Ms\(f(:,i+1) - cm*v(:,i+1) - km*x(:,i+1));
    v(:,i+1) = v(:,i+1) + gamma*dt*a(:,i+1);
    x(:,i+1) = x(:,i+1) + beta*dt^2*a(:,i+1);
    
end

x = S*x;
v = S*v;
a = S*a;
