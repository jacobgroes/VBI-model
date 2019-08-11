function [b] = IntMat(pos,X,X3,T,bg,e,intw,l,la,S,nt)
%**************************************************************************
% File: IntMat.m
%   Creates the modal vertical interaction matrix 
% Syntax:
%   b = IntMat(pos,X,X3,T,bg,e,intw,l,la,S,nt)
% Input:
%   pos : Wheel positions
%   X   : Node coordinate array
%   X3  : Third node coordinates
%   T   : Topology array
%   bg  : Initialised interaction matrix
%   e   : Train CG eccentricity to bridge shear center
%   intw: Index of interacting wheels
%   l   : Element lengths
%   la  : Accumulated element lengths
%   S   : Matrix of mode-shapes
%   nt  : Number of trains
% Output:
%   b   : Modal vertical interaction matrix
% Date:
%   Version 1.0    10.07.19
%**************************************************************************

% Find x-position of left interaction node
for i=1:length(pos)
in = find(min(abs(pos(i)-la))==abs(pos(i)-la));

in = in(1);
if pos(i)-la(in,1)<0
    in=in-1;
end

% Normalized distance from load to interaction nodes. In case in is equal
% to the last bridge node, that node is identified as the right node of the
% last element.
if in==length(X(:,1))
    s = 1; in = in-1; 
else
    s = (pos(i)-la(in,1))/l(in); 
end

% Element inclination to horizontal
theta = asin((X(T(in,2),3) - X(T(in,1),3))/l(in)); 

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

% Local element force distributions from each interacting wheel
be = N'*H;

% Third node corresponding to current element
Xe  = X(T(in,1:2),:);
X3e = X3(in,:);

% Transformation matrix
Ae = Aebeam(Xe,X3e);

% Force distribution to global dof's
be = Ae'*be;

% The interaction matrix b is a n x 2 x nt tensor, where n is the number of
% all dofs and nt is the number of wagons. 
bg(6*(in-1)+1:6*(in+1),intw(i)) = be;

end

% Modal interaction matrix
for i=1:nt
    b(:,:,i)=S'*bg(:,:,i);
end
