function plotelemdisp(T,X,Ue)
%********************************************************
% File: plotelemdisp.m
%   Plots elements in topology matrix T with
%   coordinate matrix X. Uses linear line segment
%   between all data.
% Syntax:
%   plotelem(X,T)
% Input:
%   T : Element topology matrix
%   X : Node coordinate matrix
%  ue : Element displacement vector
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Initial geometry
plot3(X(:,1),X(:,2),X(:,3),'k.','markersize',15)
hold on
for j = 1:size(T,1)
    plot3([X(T(j,1),1)
        X(T(j,2),1)],[X(T(j,1),2)
        X(T(j,2),2)],[X(T(j,1),3)
        X(T(j,2),3)],'k--','linewidth',1)
end

% Deformed geometry.
ndata = size(Ue,2);

for j = 1:size(T,1)
    X1 = X(T(j,1),:);
    a0 = (X(T(j,2),:)-X(T(j,1),:));
    for k = 1:ndata
        s = (k-1)/(ndata-1);
        Xd(k,:) = X1+a0*s+[Ue(j,k,1) Ue(j,k,2) Ue(j,k,3)];
    end
    plot3(Xd(1:ndata,1),Xd(1:ndata,2),Xd(1:ndata,3),'k-')
end

hold off
axis('equal')
axis('off')
