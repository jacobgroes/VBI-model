function [b] = IntMatH(pos,X,X3,T,bg,e,intw,l,la,S,nt,ll,lb)
%**************************************************************************
% File: IntMatH.m
%   Creates the horizontal interaction matrix 
% Syntax:
%   b = IntMatH(pos,X,X3,T,bg,e,intw,l,la,S,nt,ll,lb)
% Input:
%   pos : Positions of wheels in contact with the bridge 
%   X   : Node coordinate array
%   X3  : Third node coordinates
%   T   : Topology array
%   e   : Vertical train CM eccentricity to bridge shear center
%   bg  : Initialised interaction matrix
%   intw: Wheel numbers corresponding to the elements in pos
%   l   : Element lengths
%   la  : Accumulated element lengths
%   S   : Matrix of mode-shapes
%   nt  : Number of trains
%   ll  : Distance from train CG to front and rear wheel
%   lb  : Distance from train CG to left and right wheel
% Output:
%   b   :  Modal horizontal interaction matrix
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

% Present vehicle
vno = ceil(intw(i)/4);
% Present wheel
wno = ((1+intw(i)/4)-vno)*4;

% Weighting of equivalent wheel mass
if wno == 1
    w = ll(2,1,vno)*lb(2,1,vno)/(sum(ll(:,1,vno))*sum(lb(:,1,vno)));
elseif wno == 2
    w = ll(1,1,vno)*lb(2,1,vno)/(sum(ll(:,1,vno))*sum(lb(:,1,vno)));
elseif wno == 3
    w = ll(2,1,vno)*lb(1,1,vno)/(sum(ll(:,1,vno))*sum(lb(:,1,vno)));
elseif wno == 4
    w = ll(1,1,vno)*lb(1,1,vno)/(sum(ll(:,1,vno))*sum(lb(:,1,vno)));
end

% Normalized distance from load to interaction nodes. In case in is equal
% to the last bridge node, that node is identified as the right node of the
% last element.
if in==length(X(:,1))
    s = 1; in = in-1; 
else
    s = (pos(i)-la(in,1))/l(in); 
end 

% Shape-functions  
N31 = 2*s^3-3*s^2+1;   N32 = -2*s^3+3*s^2;           N41 = 1-s;     
N42 = s;               N51 = l(in)*(s^3-2*s^2+s);    N52 = l(in)*(s^3-s^2);    

% Interpolation matrix
N = [ 0 N31 0  0  0 N51 0 N32 0  0  0 N52
      0  0  0 N41 0  0  0  0  0 N42 0  0];

% Load component vector
H = [w -w*e]';

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

