%********************************************************
% File: DynFrame.m
%   Driver for linear dynamic finite element analysis of
%   spatial elastic beam structure. 
%   
%   Developed as an extension to the original MaxiFrame.m for static
%   analysis, created for the course 41207: Thin-Walled Beam Structures at 
%   DTU by Jan Becker Høgsberg
% Structure Input: (from datafile to memory)
%   X :  Node coordinate array
%   T :  Topology array
%   G :  Material property array
%   C :  Boundary condition array
%   P :  Prescribed nodal loads
%   p :  Prescribed element loads
%   X3:  3rd node coordinate array
% Input for dynamic analysis: (from datafile to memory)
% MaxiFrame output:
%   Plot of initial and displaced geometry
%   U   : Global nodal displacement vector
%   Sen : Element sectional force array
%   R   : Element nodal force array
% DynFrame output:
%   Response(x,v,a) with index n, m and nm for newmark, modal analysis and
%   newmark in modal coordinates, respectively.
% Date:
%   Version 1.0        21.07.2019
%********************************************************
% Clear memory
clear all    % Clear all predefined variables
close all    % Close all windows and figures
clc          % Clear command window

% Add path with input data
addpath('examples')
% Add path with functions
addpath('Functions')
% Add path with plot functioncs
addpath('Plot functions')

% Read input data
%Ex_cantilever;
Ex_simplebridge
%Ex_IntMatVer
%Ex_contbridge
%Ex_CurvingFixedBridge;

% Initialise system variables
nn   = size(X,1);               % No of system nodes
ne   = size(T,1);               % No of system elements
dof  = 2*size(X,2);             % No of degrees of freedom per node
ndof = 2*size(X,1)*size(X,2);   % No of system equations

% Initial geometry
figure(1), clf
plotelem(X,T,1);

% Initialize system vectors and matrices
K = zeros(ndof,ndof);   % Global stiffness matrix
M = zeros(ndof,ndof);   % Global stiffness matrix
F = zeros(ndof,1);      % Global load vector
U = zeros(ndof,1);      % Global displacement vector
R = zeros(2*dof,ne);    % Global element nodal force array

% STATIC ANALYSIS ---------------------------------------------------------

% Load vector from nodal loads
if exist('P','var')
F = setload(F,P,dof);
end

% Load vector from distributed loads
if exist('p','var')
F = setdistload(F,X,T,X3,p);
end

% Stiffness matrix
K0    = Kbeam(K,T,X,G,X3);
[K,F] = setbound(K0,F,C,dof);

% Solve equations
U = K\F;
V = reshape(U,dof,size(X,1))';  % Organise displacements

% Element displacements
nu  = 11;  % Number of data points
Uen = Ubeam(T,X,X3,U,nu);
if exist('p')  % Add displacements from distributed load
   Uen = Uen + Ubeamp(T,X,G,X3,p,nu);
end

% Element sectional forces
ns  = 11;  % Number of data points
Sen = Sbeam(T,X,G,X3,U,ns);
if exist('p')  % Add forces from distributed load
   Sen = Sen + Sbeamp(T,X,X3,p,ns);
end

% Global nodal forces
R = K0*U;

% Plot deformed shape
figure(2), clf
plotelemdisp(T,X,Uen)

% DYNAMIC ANALYSIS --------------------------------------------------------

% Read parameters for dynamic analysis. (Separate input data files)
timepar_cantilever
%timepar_simplebridge

% Mass matrix
M = Mbeam(M,T,X,G,X3);

% All dof's
dofs = (1:ndof)';

% Constrained dof's
cdof = zeros(length(C(:,1)),1);
for i=1:size(cdof,1)
    cdof(i) = (C(i,1)-1)*dof + C(i,2);
end

% Logical statement vector of un-constrained dof's
udof = true(ndof,1);
for i=1:length(cdof)
    udof(dofs == cdof(i)) = false;
end

% Modal analysis
[S,xm,vm,am,tm,mm,km,f,r0,s0,omega] = modalanalysis(K,M,zeta,x0,...
    v0,udof,Fd,N,dt);

% Newmark time integration
[xn,vn,an,tn] = newmark(K,M,x0,v0,a,b,gamma,beta,udof,ndof,dt,N,Fd);

% Newmark time integration in modal coordinates
[xnm,vnm,anm,tnm] = Mnewmark(km,mm,f,zeta,r0,s0,N,dt,gamma,beta,S);

% OUTPUT EXTRACTION -------------------------------------------------------
% Include constrained dof's in S for modal expansion
c = 0;
SM = zeros(ndof,sum(udof));
for i=1:ndof
    if udof(i) == true
        c = c+1;          % Iterator
        SM(i,:) = S(c,:);
    end
end

% Output setup for modal expansion
u = zeros(nn,10,length(SM(1,:)));
for i=1:length(SM(1,:))
    u(:,1,i) = SM(1:6:end,i);
    u(:,2,i) = SM(2:6:end,i);
    u(:,3,i) = SM(3:6:end,i);
    u(:,4,i) = SM(4:6:end,i);
    u(:,5,i) = SM(5:6:end,i);
    u(:,6,i) = SM(6:6:end,i);
    u(:,7,i) = [1:1:nn]';
    u(:,8,i) = X(:,1);
    u(:,9,i) = X(:,2);
    u(:,10,i) = X(:,3);
end

% Natural freqencies [Hz]
f = omega/(2*pi);

% Bridge length
L = sum(sqrt((X(2:end,1)-X(1:end-1,1)).^2+...
    (X(2:end,2)-X(1:end-1,2)).^2+(X(2:end,3)-X(1:end-1,3)).^2));

% Extract output
save('ModExpInput','u','f','L')
%save('SimpleBridge','u','f','L')