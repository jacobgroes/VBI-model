function [b] = IntMat(pos,X,X3,T,bg,ew,intw,lb,l,la,S,nt)
%**************************************************************************
% File: IntMat.m
%   Creates the modal vertical interaction matrix 
% Syntax:
%   b = IntMat(pos,X,X3,T,bg,ew,intw,lb,l,la,S,nt)
% Input:
%   pos : Positions of wheels in contact with the bridge 
%   X   : Node coordinate array
%   bg  : Initialised interaction matrix
%   ew  : Train CG eccentricity to bridge shear center
%   intw: Wheel numbers corresponding to the elements in pos
%   lb  : Distances from train CG to left and right wheel suspension
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

% Identify wagon corresponding to the wheel intw(i)
iw = floor(intw(i)/4-0.001)+1;

% Excentricity definition. The excentricity is defined as the horizontal
% distance from the shear center of the bridge to the center of gravity of 
% the train. The load positions at the wheels and thus the excentricities 
% should be described as el = e+bl and er = e-br respectively. OBS: This 
% requires train models with all four wheels. The modulo operation 
% (Remainder after division) is used to identify the wheels.
if mod(intw(i),4) == 0 || mod(intw(i)+1,4) == 0
    e = ew(iw)+lb(2,iw);
else
    e = ew(iw)-lb(1,iw);
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

% The interaction matrix b is a n x 4 x nt tensor, where n is the number of
% all dofs and nt is the number of wagons.
bg(6*(in-1)+1:6*(in+1),intw(i)) = be;

end

% Modal interaction matrix
for i=1:nt
    b(:,:,i)=S'*bg(:,:,i);
end
