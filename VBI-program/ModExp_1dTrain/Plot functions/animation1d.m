function [] = animation1d(scalev,rr,scales,scaleh,scaley,dispw,rw,u,X,T,animation,z,xx,nno,N,np,posf,xxv,xxh,e,dt)
%**************************************************************************
% File: animation1d.m
%   Creates animation of the vertical, horisontal and torsional bridge
%   displacement along with scaled spring displacements of chosen
%   suspension springs.
% Syntax:
%   [] = animation1d(scalev,rr,scales,scaleh,scaley,...
%   dispw,rw,u,X,T,animation,z,xx,nno,N,np,posf,xxv,xxh,e,dt)
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
%   np  : Number of contact points
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

    
sp = max(max(max(abs(z(:,:,:)))))*1.2; %Springs scaling according to spring displacement
sph = [sp 11/12*sp 9/12*sp 7/12*sp 5/12*sp 3/12*sp 1/12*sp 0]';

% Wether bridge geometry prevent vehicle illustration
log1 = X(1,2)==X(:,2); log2 = X(1,3)==X(:,3);
if sum(log1) < nno
    np=0;
elseif sum(log2) < nno
    np=0;
end

%v = VideoWriter('peaks.avi');
%open(v);

if animation == 1

% Animation plot
for i=1:N+1
    
    % VERTICAL DISPLACEMENT PLOT

    figure(20); clf; grid on;
    subplot('Position',[0.05 0.4 0.60 0.50]);
    [Xd,nrp] = plotDofM(X,T,D,nel,xx,i,L,scalev); % Plots deformed geometry of the bridge in every time step
    
    hold on
    
    for j=1:np

    poss(1,:) = (posf(j,i) + [0 -Lt/rr Lt/rr -Lt/rr Lt/rr -Lt/rr Lt/rr 0])'; % Spring position 
  
    spv = scales*sph*(1+z(j,i)/sp)+xxv(j,i);% Further spring scaling according to bridge displacement
    
    plot(poss,spv,'k','Linewidth',1) %Plot spings
    plot([posf(j,i) posf(j,i)],[spv(1) spv(1)+max(max(abs(xx)))*0.1],'b','Linewidth',15) %Plot masses
    
    xlim([min(X(:,1))-abs(X(end,1))*0.5 max(X(:,1))+abs(X(end,1))*0.75])
    ylim([-max(max(max(abs(xx(3:6:end,:)))))*1.2*scalev+min(X(:,3)) (max(max(max(abs(xx(3:6:end,:)))))+sp)*scalev+max(X(:,3))])

    end
    
    title({'Vertical disp. (pos. y-dir.) - disp. scale: ' num2str(scalev, '%10.2e')})
    
    hold off
    
    % HORIZONTAL DISPLACEMENT PLOT
    
    if max(max(abs(xx(2:6:end,:)))) > 10^(-9) % Only allows animation for horizontal displacements above 1e(-9)
    subplot('Position',[0.05 0.06 0.60 0.25])
    [~,~] = plotDofMH(X,T,D,nel,xx,i,L,scaleh); % Plots deformed geometry of the bridge in every time step 
    hold on
    
    for j=1:np
    poss3(j,:) = (posf(j,:))'; % Spring position
    
    plot(poss3(j,i),xxh(j,i)*scaleh,'kx','Linewidth',3) %Plots spring 
    
    end
    ylim([-max(max(max(abs(xx(2:6:end,:)))))*1.2*scaleh+min(X(:,2)) max(max(max(abs(xx(2:6:end,:)))))*scaleh+max(X(:,2))])
    xlim([min(X(:,1))-abs(X(end,1))*0.5 max(X(:,1))+abs(X(end,1))*0.75])
    title({'Transverse disp. (neg. z-dir.) - disp. scale: ' num2str(scaleh, '%10.2e')})
    hold off
    
    end
    
    % MOVING CROSS SECTION PLOT
    
    subplot('Position',[0.70 0.4 0.25 0.50])
    if np~=0
    poss1(1,:) = (e + [0 -Lt/rw Lt/rw -Lt/rw Lt/rw -Lt/rw Lt/rw 0]);%+xxh(1,i,1); % Spring position 
        
    xd1 = find(min(abs(posf(dispw,i)-Xd(1,:)))==abs(posf(dispw,i)-Xd(1,:))); % Vertical bridge deformation at interaction point
    
    % Displacements away from bridge shear center from x-rotation
    rotin = xx(4:6:end,i);
    it = 1;
    rot = zeros(nrp*nel,1);
    for j = 1:nel
        rot(it:j*nrp,1) = [rotin(j) rotin(j)+(rotin(j+1)-rotin(j))*(1:(nrp-1))/(nrp-1)]';
        it = nrp*j+1;
    end
    
    spv1 = scales*sph*(1+z(dispw,i)/sp)+xxv(dispw,i)+rot(xd1(1))*e; %Deformed spring height
    
    plot(poss1+xxh(dispw,i)*scaley,spv1,'k','Linewidth',1) %Plots spring 1
    hold on
    plot(xxh(dispw,i)*scaley,xxv(dispw,i),'mx','Linewidth',3) % Plots bridge shear center
    plot([-e 0 e] + xxh(dispw,i)*scaley,[(xxv(dispw,i)-rot(xd1(1))*e) xxv(dispw,i) xxv(dispw,i)+rot(xd1(1))*e],'b-','Linewidth',1) %Plot bridge deck
    plot([e + xxh(dispw,i)*scaley,e + xxh(dispw,i)*scaley],[spv1(1) spv1(1)+max(max(abs(xx)))*0.1],'b','Linewidth',20) %Plot masses
    xlim([-(e+max(max(abs(xx)))*(1+e/100))*1.5 (e+max(max(abs(xx)))*(1+e/100))*1.5])
    ylim([-max(max(max(abs(xx(3:6:end,:)))))*1.2+min(X(:,3)) max(max(max(abs(xx(3:6:end,:)))))+sp+max(X(:,3))])

    title({'Cross section disp.','tranverse disp. scale: ' num2str(scaley, '%10.2e')});
    end
    
    set(gcf, 'Position',  [10, 150, 1500, 650])
    [i*dt, N*dt+dt] % Time-step counter
    
    %H = getframe(gcf);
    %writeVideo(v,H)
    %set(gcf, 'Position',  [10, 150, 1500, 650])
end

end

%close(v)
