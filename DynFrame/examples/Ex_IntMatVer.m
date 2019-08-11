%********************************************************
% File: Ex_IntMatVer.m
%   Input data file for interaction matrix verification.
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

% Node coordinate array
X(1,:) = [0 0 0];
X(2,:) = [1 0 0.5];
X(3,:) = [2 0 1];
X(4,:) = [3 0 1.5];
X(5,:) = [4 0 2];
% number of nodes
nn=length(X(:,1));

% Coordinates of third nodes: X3 = [ x y z ], 
X3 = [X(:,1) -0.5*ones(nn,1) X(:,3)];
%X3 = [1 -0.5 0];

% Element topology: T = [ node1 node2 propno node3(X3) ],
% T = [ x1  x2  1  1 ]
T = ones(nn-1,4);
for i=1:nn-1
    T(i,1) = i;
    T(i,2) = i+1;
end

% Element stiffness properties: G = [ E A Iz Iy G K rho Ip ],
G = [ 200e9 0.4 0.022 0.0085 80e9 0.07 7850 0.030 ];

% Nodal load: P = [ node 'dof' value ]
l = sqrt((X(2:end,1)-X(1:end-1,1)).^2+(X(2:end,2)-X(1:end-1,2)).^2+...
    (X(2:end,3)-X(1:end-1,3)).^2);

e=1; s = 0; in = 3;
%e=1; s = 0.5; in = 2;

theta = asin((X(in+1,3) - X(in,3))/l(in)); 

P = [ in  1 0
      in  2 0
      in  3 0
      in  4 0
      in  5 0
      in  6 0
      in+1  1 0
      in+1  2 0
      in+1  3 0
      in+1  4 0
      in+1  5 0
      in+1  6 0];
  
% Shape-functions
N11 = 1-s;                N12 = s;              N21 = -6/l(in)*(-s^2+s);   
N22 = 6/l(in)*(-s^2+s);   N31 = 2*s^3-3*s^2+1;  N32 = -2*s^3+3*s^2;  
N41 = 1-s;                N42 = s;              N51 = -l(in)*(s^3-2*s^2+s); 
N52 = -l(in)*(s^3-s^2);   N61 = 3*s^2-4*s+1;    N62 = 3*s^2-2*s;

% Interpolation matrix
N = [N11 0  0  0  0  0 N12 0  0  0  0  0
      0 N21 0  0  0 N61 0 N22 0  0  0 N62
      0  0 N31 0 N51 0  0  0 N32 0 N52 0 
      0  0  0 N41 0  0  0  0  0 N42 0  0];

% Load component vector
H = [sin(theta) -sin(theta)*e cos(theta) cos(theta)*e]';

Xe  = X(T(in,1:2),:);
X3e = X3(in+1,:);
Ae = Aebeam(Xe,X3e);

%P(:,3) = Ae'*N'*H;

pp = [0 0 1 e*1 0 0]';  
P(1:6,3) = pp;

% Boundary conditions: C = [node 'dof' value ] 
C = [ 1  1  0 
      1  2  0 
      1  3  0
      1  4  0
      1  5  0
      1  6  0
      T(end,2)   1  0
      T(end,2)   2  0
      T(end,2)   3  0
      T(end,2)   4  0
      T(end,2)   5  0
      T(end,2)   6  0];
 

  