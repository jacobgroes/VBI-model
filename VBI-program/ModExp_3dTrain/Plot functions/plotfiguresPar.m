function [] = plotfiguresPar(xtr,vtr,atr,Fw,vtpar,plotfig)

if plotfig == 1

% Plot example for the parametric analysis (2). Vertical train acceleration
% vs. train velocity

figure(1)
for i=1:length(vtpar)
    atrpar(i) = max(max(abs(atr(:,:,i))));
end
plot(vtpar,atrpar,'b-*')
xlabel('$v_t$ [m/s]','Interpreter','latex')
ylabel('$a_{z}$ [m/s$^2$]','Interpreter','latex')
title('Max. vertical acc. of all vehicle bodies','Interpreter','latex')
axis([vtpar(1) vtpar(end) min(atrpar) max(atrpar)*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
%box off

figure(2)
for i=1:length(vtpar)
    xtrpar(i) = max(max(abs(xtr(:,:,i))));
end
plot(vtpar,xtrpar,'k-*')
xlabel('$v_t$ [m/s]','Interpreter','latex')
ylabel('$x_{z}$ [m]','Interpreter','latex')
title('Max. vertical disp. of all vehicle bodies','Interpreter','latex')
axis([vtpar(1) vtpar(end) min(xtrpar) max(xtrpar)*1.2])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)
%box off

figure(3)
for i=1:length(vtpar)
    FwparMax(i) = max(max(Fw(:,:,i)));
end
plot(vtpar,FwparMax,'k.-')
xlabel('$v_t$ [m/s]','Interpreter','latex')
ylabel('$F_{w,1}$ [N]','Interpreter','latex')
title('Max. contact force of all vehicles','Interpreter','latex')
%legend({'Front left wheel','Rear left wheel','Front right wheel','Rear right wheel'},'Interpreter','latex','Location','best')
%legend boxoff
axis([vtpar(1) vtpar(end) min(FwparMax)*1.2 max([max(FwparMax),0])])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)   

figure(4)
for i=1:length(vtpar)
    FwparMin(i) = min(min(Fw(:,:,i)));
end
plot(vtpar,FwparMin,'b.-')
xlabel('$v_t$ [m/s]','Interpreter','latex')
ylabel('$F_{w,1}$ [N]','Interpreter','latex')
title('Min. contact force of all vehicles','Interpreter','latex')
%legend({'Front left wheel','Rear left wheel','Front right wheel','Rear right wheel'},'Interpreter','latex','Location','best')
%legend boxoff
axis([vtpar(1) vtpar(end) min(FwparMin)*1.2 max([max(FwparMin),0])])
set(gca,'TickLabelInterpreter','latex')
set(gca,'Fontsize',14)   

end