%********************************************************
% File: Ex_contbridge.m
%   Input data file for two-span continuous bridge.
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
nn=9+2*8; % number of nodes
Lb = 200; % bridge length
X = zeros(nn,3);
for i=1:nn
    X(i,1) = (i-1)*Lb/(nn-1);
end

% Coordinates of third nodes: X3 = [ x y z ], 
X3 = [ 0.5  -0.5  0 ];

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
%P = [ 2  1 -1 ];
  
% Boundary conditions: C = [node 'dof' value ]  
C = [ 1  1  0 
      1  2  0 
      1  3  0
      1  4  0
% %      9  1  0
%       9  2  0 
%       9  3  0 
%       9  4  0
% %      17  1  0
%       17  2  0 
%       17  3  0 
%       17  4  0 
% %      25  1  0
%       25  2  0
%       25  3  0 
%       25  4  0 
% %      33  1  0
%       33  2  0 
%       33  3  0
%       33  4  0 
      13  2  0 
      13  3  0
      13  4  0 
      T(end,2)  2  0 
      T(end,2)  3  0
      T(end,2)  4  0
      T(end,2)  5  0
      T(end,2)  6  0];
 

  