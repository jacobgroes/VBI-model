function [Xd,nrp] = plotDofM(X,T,D,nel,xp,o,L,scalev)
%**************************************************************************
% File: plotDofM.m
%   Plots the vertical bridge deformation
% Syntax:
%   [Xd,nrp] = plotDofM(X,T,D,nel,xp,o,L,scalev)
% Input:
%   X   : Node coordinate array
%   T   : Node topology array
%   D   : Dof identifier
%   nel : Number of elements
%   xp  : Horisontal nodal bridge deformations
%   o   : Current time increment
%   L   : Element lengths
%   scalev: Vertical displacement scaling factor
% Output:
%   Xd  : Deformed bridge geometry 
%   nrp : Default number of interpolation points
% Date:
%   Version 1.0    20.06.19
%************************************************************************** 
% number of element plot points 
nrp=11;
Xd=zeros(2,nrp*nel);
    hold on
for el = 1:nel
  plot(X(T(el,:),1),X(T(el,:),3),'k:')
  Xs=zeros(2,nrp);
  for i=1:nrp
    s=(i-1)/(nrp-1);
    NN=[1-s             0               0 s           0            0;
       0   2*s^3-3*s^2+1 -L(el)*(s^3-2*s^2+s) 0 -2*s^3+3*s^2 -L(el)*(s^3-s^2)];
    Xs(:,i)=X(T(el,1),[1,3])'*(1-s)+X(T(el,2),[1,3])'*s+scalev*NN*xp(D(el,[1,3,5,7,9,11])',o);
  end

  plot(Xs(1,:),Xs(2,:),'b-');
  
  hold on
  xlim([0 X(end,1)])
  if max(max(max(abs(xp)))) ~=0
  ylim([-max(max(max(abs(xp))))*1.2 max(max(max(abs(xp))))*1])
  end
  Xd(1,el*nrp-(nrp-1):nrp*el) = Xs(1,:);
  Xd(2,el*nrp-(nrp-1):nrp*el) = Xs(2,:);
end
hold off
end