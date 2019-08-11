function [] = animation2d(scalev,rr,scales,scaleh,scaley,dispyw,dispw,rw,u,X,T,animation,z,xx,nno,N,nt,posf,xxv,xxh,e,dt)
%**************************************************************************
% File: animation2d.m
%   Creates animation of the vertical, horisontal and torsional bridge
%   displacement along with scaled spring displacements of chosen
%   suspension springs.
% Syntax:
%   [] = animation2d(scalev,rr,scales,scaleh,scaley,dispyw,...
%   dispw,rw,u,X,T,animation,z,xx,nno,N,nt,posf,xxv,xxh,e,dt)
% Input:
%   Scalings: scalev, rr, scales, scaleh, scaley, rw (look in ModExp for
%   explaination)
%   Choice of displayed springs: dispyw, dispw (look in ModExp for
%   explaination)
%   u   : Input tensor
%   X   : Node coordinate array
%   T   : Node topology array
%   animation: animation == 1 => video, animation ~= 1 => no video
%   z   : Relative suspension spring displacements
%   xx  : Nodal bridge displacements
%   nno : Number of nodes
%   N   : Number of time-steps
%   nt  : Number of trains
%   posf: Contact point position matrix
%   xxv : Vertical bridge displacements at contact points
%   xxh : Horisontal bridge displacements at contact points
%   e   : Train CG eccentricity to bridge shear center
%   dt  : Time increment size
% Output:
%   Animation
% Date:
%   Version 1.0    20.06.19
%**************************************************************************
% Global dof's 
nel=length(u(:,1,1))-1;
Del = [-5 -4 -3 -2 -1 0 1 2 3 4 5 6];
for el = 1:nel
D(el,:) = Del(1,:)+6;
Del = D(el,:);
end

% Animation plot [x z phi_y] with moving boxes
L = sqrt((X(2:end,1)-X(1:end-1,1)).^2+(X(2:end,2)-X(1:end-1,2)).^2+(X(2:end,3)-X(1:end-1,3)).^2); %Element lengths
Lt = sum(L); %Total bridge length

sp = max(max(max(abs(z(:,:,:)))))*1.2; 
sph = [sp 11/12*sp 9/12*sp 7/12*sp 5/12*sp 3/12*sp 1/12*sp 0]';

log1 = X(1,2)==X(:,2); log2 = X(1,3)==X(:,3);
if sum(log1) < nno
    nt=0;
elseif sum(log2) < nno
    nt=0;
end

if animation == 1

for i=1:N+1
    
    % VERTICAL DISPLACEMENT PLOT

    figure(20); clf; grid on;
    subplot('Position',[0.05 0.4 0.60 0.50]);
    [Xd,nrp] = plotDofM(X,T,D,nel,xx,i,L,scalev); % Plots deformed geometry of the bridge in every time step 
    hold on
    
    for j=1:nt
    pp = [1 2]';
    poss(:,:,j) = (posf(pp,i,j) + [0 -Lt/rr Lt/rr -Lt/rr Lt/rr -Lt/rr Lt/rr 0])'; % Spring position
    
    spv1 = scales*sph*(1+z(1,i,j)/sp)+xxv(1,i,j);%Xd(2,xd1); %Deformed spring height 1
    spv2 = scales*sph*(1+z(2,i,j)/sp)+xxv(2,i,j);%Xd(2,xd2); %Deformed spring height 2
    
    
    plot(poss(:,1,j),spv1*scalev,'k','Linewidth',1) %Plots spring 1
    plot(poss(:,2,j),spv2*scalev,'r','Linewidth',1) %Plots spring 2
    plot([poss(1,1,j) poss(1,2,j)],[spv1(1) spv2(1)]*scalev,'b','Linewidth',5) %Plots masses
    
    end
    
    xlim([min(X(:,1))-abs(X(end,1))*0.5 max(X(:,1))+abs(X(end,1))*0.75])
    ylim([-max(max(max(abs(xx(3:6:end,:)))))*1.2*scalev+min(X(:,3)) (max(max(max(abs(xx(3:6:end,:)))))+sp)*scalev+max(X(:,3))])
    
    title({'Vertical disp. (pos. y-dir.) - disp. scale: ' num2str(scalev, '%10.2e')})
    
    hold off
    
    % HORIZONTAL DISPLACEMENT PLOT
    
    if max(max(abs(xx(2:6:end,:)))) > 10^(-9)
    subplot('Position',[0.05 0.06 0.60 0.25])
    [Xdh,~] = plotDofMH(X,T,D,nel,xx,i,L,scaleh); % Plots deformed geometry of the bridge in every time step 
    hold on
    
    for j=1:nt
    pp = [1 2]';
    poss3(:,:,j) = (posf(pp,i,j))'; % Spring position
    
    plot(poss3(:,1,j),xxh(1,i,j)*scaleh,'kx','Linewidth',3) %Plots spring 1
    plot(poss3(:,2,j),xxh(2,i,j)*scaleh,'rx','Linewidth',3) %Plots spring 2
    
    end
    ylim([-max(max(max(abs(xx(2:6:end,:)))))*1.2*scaleh+min(X(:,2)) max(max(max(abs(xx(2:6:end,:)))))*scaleh+max(X(:,2))])
    xlim([min(X(:,1))-abs(X(end,1))*0.5 max(X(:,1))+abs(X(end,1))*0.75])
    title({'Transverse disp. (neg. z-dir.) - disp. scale: ' num2str(scaleh, '%10.2e')})
    hold off
    end
    
    % MOVING CROSS SECTION PLOT
    
    subplot('Position',[0.70 0.4 0.25 0.50])
    if nt~=0
    poss1(:,:,1) = (e + [0 -Lt/rw Lt/rw -Lt/rw Lt/rw -Lt/rw Lt/rw 0]);%+xxh(1,i,1); % Spring position front 1
        
    xd1 = find(min(abs(posf(1,i,dispw)-Xd(1,:)))==abs(posf(1,i,dispw)-Xd(1,:))); % Vertical bridge deformation at interaction point 1

    rotin = xx(4:6:end,i);
    it = 1;
    rot = zeros(nrp*nel,1);
    for j = 1:nel
        rot(it:j*nrp,1) = [rotin(j) rotin(j)+(rotin(j+1)-rotin(j))*(1:(nrp-1))/(nrp-1)]';
        it = nrp*j+1;
    end
    
    spv1 = scales*sph*(1+z(dispyw,i,dispw)/sp)+xxv(1,i,dispw)+rot(xd1(1))*e; %Deformed spring height zf1
    
    plot(poss1+xxh(dispyw,i,dispw)*scaley,spv1,'k','Linewidth',1) %Plots spring 1
    hold on
    plot(xxh(dispyw,i,dispw)*scaley,xxv(dispyw,i,dispw),'mx','Linewidth',3)
    plot([-e 0 e] + xxh(dispyw,i,dispw)*scaley,[(xxv(dispyw,i,dispw)-rot(xd1(1))*e) xxv(dispyw,i,dispw) xxv(dispyw,i,dispw)+rot(xd1(1))*e],'b-','Linewidth',1) %Plot bridge deck
    plot([e + xxh(dispyw,i,dispw)*scaley,e + xxh(dispyw,i,dispw)*scaley],[spv1(1) spv1(1)+max(max(abs(xx)))*0.1],'b','Linewidth',20) %Plots masses
    xlim([-(e+max(max(abs(xx)))*(1+e/100))*1.5 (e+max(max(abs(xx)))*(1+e/100))*1.5])
    ylim([-max(max(max(abs(xx(3:6:end,:)))))*1.2+min(X(:,3)) max(max(max(abs(xx(3:6:end,:)))))+sp+max(X(:,3))])

    title({'Cross section disp.','tranverse disp. scale: ' num2str(scaley, '%10.2e')});
    end
    
    set(gcf, 'Position',  [10, 150, 1500, 650])
    [i*dt, N*dt+dt]
end

end