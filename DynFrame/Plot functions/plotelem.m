function plotelem(X,T,N)
%********************************************************
% File: plotelem.m
%   Plots elements in topology matrix T with
%   coordinate matrix X. Uses linear line segment
%   between all nodes.
% Syntax:
%   plotelem(X,T)
% Input: 
%   X : Node coordinate array 
%   T : Element topology array
%   N : Node number parameter
% Date:
%   Version 1.0    10.02.11
%********************************************************

% Plot nodes
plot3(X(:,1),X(:,2),X(:,3),'k.','markersize',15)
hold on

% Plot elements
for j = 1:size(T,1)
      plot3([X(T(j,1),1)
          X(T(j,2),1)],[X(T(j,1),2)
          X(T(j,2),2)],[X(T(j,1),3)
          X(T(j,2),3)],'k-','linewidth',1)
end

% Node number offset
offset = 0.035*max(max(abs(X)));

if N == 1
    % Plot node numbers
    for i = 1:size(X,1)
        text(X(i,1)+offset,X(i,2)+offset,X(i,3)+offset,int2str(i))
    end
end

hold off
axis('equal')
axis('off')