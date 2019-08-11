function [] = animation3d(scalev,dispvw,rr,scales,scaleh,scaley,dispyw,...
    dispw,rw,u,X,T,animation,z,xx,nno,N,nt,posf,xxv,xxh,e,dt,lb)
%**************************************************************************
% File: animation3d.m
%   Creates animation of the vertical, horisontal and torsional bridge
%   displacement along with scaled spring displacements of chosen
%   suspension springs.
% Syntax:
%   [] = animation3d(scalev,dispvw,rr,scales,scaleh,scaley,dispyw,...
%   dispw,rw,u,X,T,animation,z,xx,nno,N,nt,posf,xxv,xxh,e,dt,lb)
% Input:
%   Scalings: scalev, rr, scales, scaleh, scaley, rw (look in ModExp for
%   explaination)
%   Choice of displayed springs: dispvw, dispyw, dispw (look in ModExp for
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
%   lb  : CG distances to left and right wheel
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

e = e(dispw);

if animation == 1
sp = max(max(max(abs(z(:,:,:)))))*1.2; 
sph = [sp 11/12*sp 9/12*sp 7/12*sp 5/12*sp 3/12*sp 1/12*sp 0]';

log1 = X(1,2)==X(:,2); log2 = X(1,3)==X(:,3);
if sum(log1) < nno
    nt=0;
elseif sum(log2) < nno
    nt=0;
end

%v = VideoWriter('videotest2.avi');
%open(v);

for i=1:N+1
    
    % VERTICAL DISPLACEMENT PLOT
    
    figure(10); clf; grid on; 
    subplot('Position',[0.05 0.4 0.60 0.50]);
    [Xd,nrp] = plotDofM(X,T,D,nel,xx,i,L,scalev); % Plots deformed geometry of the bridge in every time step 
    
    hold on
    
    for j=1:nt
    pp = [1 2]';
    poss(:,:,j) = (posf(pp,i,j) + [0 -Lt/rr Lt/rr -Lt/rr Lt/rr -Lt/rr Lt/rr 0])'; % Spring position
    
    spv1 = scales*sph*(1+z(dispvw,i,j)/sp)+xxv(dispvw,i,j); %Deformed spring height zf1
    spv2 = scales*sph*(1+z(dispvw+1,i,j)/sp)+xxv(dispvw+1,i,j); %Deformed spring height zr1
    
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
    poss1(:,:,1) = (e-lb(1,1,dispw) + [0 -Lt/rw Lt/rw -Lt/rw Lt/rw -Lt/rw Lt/rw 0]);% Spring position front 1
    poss2(:,:,1) = (e+lb(2,1,dispw) + [0 -Lt/rw Lt/rw -Lt/rw Lt/rw -Lt/rw Lt/rw 0]);% Spring position fron 2
        
    xd1 = find(min(abs(posf(1,i,dispw)-Xd(1,:)))==abs(posf(1,i,dispw)-Xd(1,:)));% Vertical bridge deformation at interaction point 1

    rotin = xx(4:6:end,i);
    it = 1;
    rot = zeros(nrp*nel,1);
    for j = 1:nel
        rot(it:j*nrp,1) = [rotin(j) rotin(j)+(rotin(j+1)-rotin(j))*(1:(nrp-1))/(nrp-1)]';
        it = nrp*j+1;
    end
    
    spv1 = scales*sph*(1+z(dispyw,i,dispw)/sp)+xxv(dispyw,i,dispw)+rot(xd1(1))*(e-lb(1,1,dispw));% Deformed spring height zf1
    spv3 = scales*sph*(1+z(dispyw+2,i,dispw)/sp)+xxv(dispyw+2,i,dispw)+rot(xd1(1))*(e+lb(2,1,dispw));% Deformed spring height zf2
    
    plot(poss1+xxh(dispyw,i,dispw)*scaley,spv1,'k','Linewidth',1)% Plots spring 1
    hold on
    plot(poss2+xxh(dispyw+2,i,dispw)*scaley,spv3,'m','Linewidth',1)% Plots spring 2
    plot(xxh(dispyw,i,dispw)*scaley,xxv(dispyw,i,dispw),'mx','Linewidth',3)
    plot([-(e+lb(2,1,dispw)) -(e-lb(1,1,dispw)) e-lb(1,1,dispw) 0 e+lb(2,1,dispw)] + xxh(dispyw,i,dispw)*scaley,[(xxv(dispyw,i,dispw)-rot(xd1(1))*(e+lb(2,1,dispw))) (xxv(dispyw,i,dispw)-rot(xd1(1))*(e-lb(1,1,dispw))) xxv(dispyw,i,dispw)+rot(xd1(1))*(e-lb(1,1,dispw)) xxv(dispyw,i,dispw) xxv(dispyw,i,dispw)+rot(xd1(1))*(e+lb(2,1,dispw))],'b-','Linewidth',1)% Plot bridge deck
    plot([e-lb(1,1,dispw) e e+lb(2,1,dispw)] + xxh(dispyw,i,dispw)*scaley,[spv1(1) spv1(1)+(spv3(1)-spv1(1))/(lb(2,1,dispw)+lb(1,1,dispw))*lb(1,1,dispw) spv3(1)],'b','Linewidth',5) %Plots masses
    xlim([-max([abs((e+lb(2,1,dispw))),abs((e-lb(1,1,dispw)))])*1.5 max([abs((e+lb(2,1,dispw))),abs((e-lb(1,1,dispw)))])*1.5])
    ylim([-max(max(max(abs(xx(3:6:end,:)))))*1.2+min(X(:,3)) max(max(max(abs(xx(3:6:end,:)))))+sp+max(X(:,3))])

    title({'Cross section disp.','tranverse disp. scale: ' num2str(scaley, '%10.2e')});
    end
    set(gcf, 'Position',  [10, 150, 1500, 650])
    hold off
%    [i*dt, N*dt+dt]
    
    %H = getframe(gcf);
    %writeVideo(v,H)
    %set(gcf, 'Position',  [10, 150, 1500, 650])
end
end