function [] = plotfigures(t,xb,vb,ab,X,Fw,xxv,xxh,xtr,atr,z,plotfig,nno,n)
%**************************************************************************
% File: plotfigures.m
%   Plots bridge and train response. The user of the program is allowed to
%   edit input to this function.
% Syntax:
%   [] = plotfigures(t,xb,vb,ab,X,Fw,xxv,xxh,xtr,z,plotfig,nno,n)
% Input:
%   xb  : Nodal bridge displacements
%   vb  : Nodal bridge velocities
%   ab  : Nodal bridge accelerations
%   Fw  : Contact force array
%   xtr : Vertical train displacements
%   z   : Relative suspension spring displacements
%   plotfig: plotfig == 1 => figures, plotfig ~= 1 => no figures
%   nno : Number of nodes
%   n   : Number of modes included in analysis
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

if min(min(xb(5*6-4,:))) ~= 0 || max(max(xb(5*6-4,:)))
figure(3)
plot(t,xb(5*6-4,:),'k-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$x_{b,y}$ [m]','Interpreter','latex')
title('Horisontal bridge displ. at node 5','Interpreter','latex')
axis([0 t(end) min(xb(5*6-4,:))*1.2 max(xb(5*6-4,:))*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
end

if min(min(Fw(1:4,:))) ~= 0 || max(max(Fw(1:4,:)))
figure(4)
plot(t,Fw(1,:),'k-','LineWidth',1.5)
hold on
plot(t,Fw(2,:),'k--','LineWidth',1.5)
plot(t,Fw(3,:),'b-','LineWidth',1.5)
plot(t,Fw(4,:),'b--','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$F_{w,1}$ [N]','Interpreter','latex')
title('Contact forces of first vehicle','Interpreter','latex')
legend({'Front left wheel','Rear left wheel','Front right wheel','Rear right wheel'},'Interpreter','latex','Location','best')
legend boxoff
axis([0 t(end) min(min(Fw(1:4,:)))*1.2 max([max(Fw(1:4,:)),0])])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)    
end

if min(min(z(1:4,:,1))) ~= 0 || max(max(z(1:4,:,1)))
figure(5) % NOTE: gravitational displacement substracted from z
plot(t(1,:,1),z(1,:,1),'b-','LineWidth',1.5)
hold on
plot(t(1,:,1),z(2,:,1),'b--','LineWidth',1.5)
plot(t(1,:,1),z(3,:,1),'r-','LineWidth',1.5)
plot(t(1,:,1),z(4,:,1),'r--','LineWidth',1.5)
plot(t(1,:,1),xtr(1,:)-xtr(1,1),'k-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$u_{z,1}$ [m]','Interpreter','latex')
title('Vertical disp. above suspensions of 1st vehicle','Interpreter','latex')
legend({'Front left','Rear left','Front right','Rear right','Body CM'},'Interpreter','latex','Location','best')
legend boxoff
axis([0 t(end) min([z(1,:,1) z(2,:,1) z(3,:,1) z(4,:,1) xtr(1,:)-xtr(1,1)])*1.2 max([z(1,:,1) z(2,:,1) z(3,:,1) z(4,:,1) xtr(1,:)-xtr(1,1)])*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)    
end

if min(atr(1,:)) ~= 0 || max(atr(1,:))
figure(6)
plot(t,atr(1,:),'b-','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$a_{z}$ [m/s$^2$]','Interpreter','latex')
title('Vertical acc. of 1st vehicle body','Interpreter','latex')
axis([0 t(end) min(atr(1,:))*1.2 max(atr(1,:))*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
box off
end

% Deflection at contact points (First train)
if min(min(xxv(1:4,:,1))) ~= 0 || max(max(xxv(1:4,:,1)))
figure(7)
plot(t,xxv(1,:,1),'b-','LineWidth',1.5)
hold on
plot(t,xxv(2,:,1),'b--','LineWidth',1.5)
plot(t,xxv(3,:,1),'k-','LineWidth',1.5)
plot(t,xxv(4,:,1),'k--','LineWidth',1.5)
xlabel('$t$ [s]','Interpreter','latex')
ylabel('$u_{z,1}$ [m]','Interpreter','latex')
title('Vertical wheel disp. of 1st vehicle','Interpreter','latex')
legend({'Front left','Rear left','Front right','Rear right'},'Interpreter','latex','Location','best')
legend boxoff
axis([0 t(end) min([xxv(1,:,1) xxv(2,:,1) xxv(3,:,1) xxv(4,:,1)])*1.2 max([xxv(1,:,1) xxv(2,:,1) xxv(3,:,1) xxv(4,:,1)])*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)   
end

% figure(10)
% plot(t,zf,'b-')
% hold on
% plot(t,zr,'r-')



end