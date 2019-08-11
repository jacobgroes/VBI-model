%********************************************************
% File: Ex_cantilever.m
%   Input data file for a cantilever beam
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
X = [ 0  0  0
      1.25  0  0
      2.5  0  0
      3.75  0  0
%      4  0  0
      5  0  0
      6.25  0  0
      7.5  0  0
      8.75  0  0
%      9  0  0
      10 0  0];


% Coordinates of third nodes: X3 = [ x y z ], 
X3 = [ 0.5  -0.5  0 ];

% Element topology: T = [ node1 node2 propno node3(X3) ],
T = [ 1  2  1  1 
      2  3  1  1
      3  4  1  1
      4  5  1  1
      5  6  1  1
      6  7  1  1
      7  8  1  1
      8  9  1  1];
%       9  10 1  1
%       10 11 1  1];

% Element stiffness properties: G = [ E A Iz Iy G K rho Ip ],
G = [ 200e9 0.4 0.022 0.0085 80e9 0.07 7850 0.030 ];

% Nodal load: P = [ node 'dof' value ]
P = [ 2  1 -1 ];
  
% Boundary conditions: C = [node 'dof' value ]  
C = [ 1  1  0 
      1  2  0 
      1  3  0
      1  4  0 
      1  5  0 
      1  6  0 ];
 

  