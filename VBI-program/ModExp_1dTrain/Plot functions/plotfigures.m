function [] = plotfigures(t,xb,vb,ab,X,Fw,xxv,xxh,xtr,atr,z,np,plotfig)
%**************************************************************************
% File: plotfigures.m
%   Plots bridge and train response. The user of the program is allowed to
%   edit input to this function.
% Syntax:
%   [] = plotfigures(t,xb,vb,ab,X,Fw,xxv,xxh,xw,z,plotfig,nno,n,nt)
% Input:
%   t   : Time increment vector
%   xb  : Nodal bridge displacements
%   vb  : Nodal bridge velocities
%   ab  : Nodal bridge accelerations
%   Fw  : Contact force array
%   xtr : Vertical train displacements
%   z   : Relative suspension spring displacements
%   np  : Number of contact points
%   plotfig: plotfig == 1 => figures, plotfig ~= 1 => no figures
% Output:
%   Figures
% Date:
%   Version 1.0    20.06.19
%**************************************************************************
if plotfig == 1

figure(1)
plot(t,xb(5*6-3,:),'b-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$x_{b,z}$ [m]','Interpreter','latex')
title('Vertical bridge displ. at node 5','Interpreter','latex')
axis([0 t(end) min(xb(5*6-3,:))*1.2 max(xb(5*6-3,:))*1.2])

%legend({'ode45','N = 200','N = 40','N = 20'},'Interpreter','latex','Location','northeast')

set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)


figure(2)
plot(t,ab(5*6-3,:),'m-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$a_{b,z}$ [m/s$^2$]','Interpreter','latex')
title('Vertical bridge acc. at node 5','Interpreter','latex')
axis([0 t(end) min(ab(5*6-3,:))*1.2 max(ab(5*6-3,:))*1.2])

set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)

if min((xb(5*6-4,:))) ~= 0 || max((xb(5*6-4,:)))
figure(3)
plot(t,xb(5*6-4,:),'b-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$x_{b,y}$ [m]','Interpreter','latex')
title('Horisontal bridge displ. at node 5','Interpreter','latex')
axis([0 t(end) min(xb(5*6-4,:))*1.2 max(xb(5*6-4,:))*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
end

if min((Fw(1,:))) ~= 0 || max((Fw(1,:)))
figure(4)
plot(t,Fw(1,:),'r-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$F_{w,1}$ [N]','Interpreter','latex')
title('Contact force of first vehicle','Interpreter','latex')
axis([0 t(end) min(Fw(1,:))*1.2 0])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
end

if min((xxv(1,:))) ~= 0 || max((xxv(1,:)))
figure(5)
plot(t,xxv(1,:),'k-','LineWidth',1.5)
hold on
plot(t,xtr(1,:)-xtr(1,1),'b-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$u_{z}$ [m]','Interpreter','latex')
title('Vertical disp. of 1st vehicle','Interpreter','latex')
legend({'Wheel disp.','Body CM disp.'},'Interpreter','latex','Location','best')
legend boxoff
axis([0 t(end) min([xxv(1,:) xtr(1,:)-xtr(1,1)])*1.2 max([xxv(1,:) xtr(1,:)-xtr(1,1)])*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
box off
end

if min(atr(1,:)) ~= 0 || max(atr(1,:))
figure(6)
plot(t,atr(1,:),'b-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$a_{z}$ [m/s$^2$]','Interpreter','latex')
title('Vertical acc. of 1st vehicle body','Interpreter','latex')
%legend({'Body CM acc.'},'Interpreter','latex','Location','best')
%legend boxoff
axis([0 t(end) min(atr(1,:))*1.2 max(atr(1,:))*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
box off
end

% figure(7)
% plot(t,xb(5*6-2,:),'k-','LineWidth',1.5)
% xlabel('$t$ [s]','Interpreter','latex')
% ylabel('$x_{b,\phi_x}$ [rad]','Interpreter','latex')
% title('Bridge torsion at node 5','Interpreter','latex')
% axis([0 t(end) min(xb(5*6-2,:))*1.2 max(xb(5*6-2,:))*1.2])

% figure(8)
% surfc(X(:,1),t',xb(3:6:end,:)')
% ylabel('time [s]')
% xlabel('span length [m]')
% zlabel('Bridge - zmax [m]')

end
