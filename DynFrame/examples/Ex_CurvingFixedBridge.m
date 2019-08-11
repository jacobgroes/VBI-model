%********************************************************
% File: Ex_CurvingFixedBridge.m
%   Input data file for curving fixed-fixed bridge 
% Output: 
%   X  :  Node coordinate array
%   X3 :  3rd node coordinate array
%   T  :  Topology array
%   G  :  Material property array
%   C  :  Boundary condition array
%   P  :  Prescribed nodal loads
% Date:
%   Version 1.0        02.08.19
%********************************************************

% Coordinates of nodes: X = [ x y z ],
nn=19; % number of nodes
Lb = 200; % bridge length
X = zeros(nn,3);
for i=1:nn
    X(i,1) = (i-1)*Lb/(nn-1);
    X(i,2) = -2/1000*(X(i,1))^2 + 2/5*X(i,1);
end

% Coordinates of third nodes: X3 = [ x y z ], 
X3 = zeros(size(X)-1);
X3(:,[1,3]) = X(2:end,[1,3]);
X3(:,2) = X(2:end,2)-1;

% Element topology: T = [ node1 node2 propno node3(X3) ],
T = ones(nn-1,4);
for i=1:nn-1
    T(i,1) = i;
    T(i,2) = i+1;
end

% Element stiffness properties: G = [ E A Iz Iy G K rho Ip ],
%G = [ 37e9 20 600 70 15e9 130 2500 650 ];
G = [ 37e9 9.071 12.53 10.00 15.53e9 129.4 2500 22.53];

% Nodal load: P = [ node 'dof' value ]
%P = [5  4 0.5*(400+2e5)*9.81];
  
% Boundary conditions: C = [node 'dof' value ]  
C = [ 1  1  0 
      1  2  0 
      1  3  0
      1  4  0 
      1  5  0
      1  6  0
      T(end,2)  1  0
      T(end,2)  2  0 
      T(end,2)  3  0
      T(end,2)  4  0
      T(end,2)  5  0
      T(end,2)  6  0];
 

  