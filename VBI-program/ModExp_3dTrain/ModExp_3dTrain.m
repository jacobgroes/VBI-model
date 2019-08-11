%*************************************************************************
% File: ModExp_3dTrain.m
%   Driver for linear-elastic dynamic analysis of vehicle-bridge 
%   interaction. The bridge is constructed from modal expansion of a 
%   linear-elastic 3d-beam-structure in a seperate software. 
%   3d-mass-spring-damper vehicles are constructed in the input section.
% Input:
%   u: Node coordinate array, topology array and eigenvector array
%   f: Eigenfrequency array
%   L: Bridge length
% Output:
%   xb,vb,ab: Bridge response histories
%   xtr,vtr,atr: Vehicle response histories
%   Fw: Contact force histories
% Date:
%   Version 1.0        10.07.19
%*************************************************************************
clc;clear;close all; % Clear memory, close figures

% Restore default path
restoredefaultpath

% Add input path
addpath('Input')

% Add function path
addpath('Functions') 

% INPUT SECTION
% -------------------------------------------------------------------------
% Choose type of analysis. ANALYSIS = 1 computes the response from a single
% train velocity. ANALYSIS = 2 is a parametric analysis computing the 
% response history from various train velocities. 
ANALYSIS = 1;

% -------------------------------------------------------------------------
% Load eigenvectors, node coordinates and topology(u), eigenfrequencies(f),
% and bridge length(L).

% Read input data 
%load('u.mat')                  % y = 3, m = 1000 (sofistik: simple bridge)
%load('Data.mat')               % y = 6, m = 1000 (sofistik: cable stayed)
load('SimpleBridge.mat')       % y = 0, m = 1 (DynFrame)
%load('ContBridge.mat')         % y = 0, m = 1 (DynFrame)
%load('pylon.mat')              % y = 0, m = 1 (DynFrame)
%load('CurvingFixedBridge.mat') % y = 0, m = 1 (DynFrame)


% -------------------------------------------------------------------------
% ADAPTATION OF INPUT

% Unit prefix of input mode-shapes (x[m])*10^y - (theta[rad])*10^y. 
% (If in millimeter, y = 3 ... If in meter, y = 0)
y = 0;

% Normalised modal mass
m = 1;

% Number of modes included in analysis
n = 20;

% -------------------------------------------------------------------------
% INITIALISE BRIDGE PARAMETERS AND GEOMETRY

% Mass normalised mode-shapes (nno*6 x n)
S = assemModes(u,y,n);  

% Modal damping ratio for all vibration modes
zeta = 0.02;

% Natural angular frequencies
omega(:,1) = f(1:n)*2*pi;

% Number of nodes in FE-model
nno = length(u(:,1,1));

% Node coordinates
X = u(:,8:10,1);  

% Continuous beam topology
T(:,1) = 1:nno-1;
T(:,2) = 2:nno;

% Coordinates of "third element nodes" X3 = [x y z] for creating the 
% element specific 3d transformation matrix.
% The setting below works in general for bridge decks curving either
% in the x-y-plane or the x-z plane. For curvature in both planes, the
% third node matrix should be defined manually.
X3 = zeros(size(X)-1);
X3(:,[1,3]) = X(2:end,[1,3]);
X3(:,2) = X(2:end,2)-1;
% -------------------------------------------------------------------------
% TIME INTEGRATION PARAMETERS

% Weighting parameters
gamma = 0.5;
beta = 0.25;

% Time history parameters
dt = 0.02; % Time increment size
N  = 200; % Number of increments
t0 = 0; % Initial time increment
tt = linspace(t0,dt*N,N+1); % Time increment vector

% -------------------------------------------------------------------------
% VEHICHLE PARAMETERS

% Initial distances from node 1 of the bridge model to the front wheels of
% each vehicle. The number of elements in the initial vector corresponds to
% the number of vehicles in the analysis.
%initial = [0 1 2 3 4 5 6 7 8 9]'*(-22.5);
initial = [0 -22.5]';

% Number of trains 
nt = length(initial);

% Constant train velocity
vt = 100; % Fixed velocity for - ANALYSIS 1
vtpar = [30:10:300]; % Velocities for parametric study - ANALYSIS 2

% Horizontal rail center eccentricity to bridge shear center
erail = 2.0;

% Rail width
brail = 1.435;

% Horizontal train center of mass(CM) eccentricity to rail center
eCM(1) = 0.4; 
eCM(2) = 0.4;
% eCM(3) = 0.4; 
% eCM(4) = 0.4;
% eCM(5) = 0.4; 
% eCM(6) = 0.4;
% eCM(7) = 0.4; 
% eCM(8) = 0.4; 
% eCM(9) = 0.4; 
% eCM(10) = 0;

% Horizontal train CM eccentricity to bridge shear center
e = erail+eCM;

% Vertical train CM eccentricity to bridge shear center (for all trains)
eh = 1;

% Train mass properties Mt(:,:,j) = [mt Jx Jy]', j = 1,2,..nt
mt = 50.99e3; % Train mass
Jx = 154.8e3; % Train mass moment of inertia about the x-axis
Jy = 195.9e4; % Train mass moment of inertia about the y-axis
Mt(:,:,1) = [mt Jx Jy]';
Mt(:,:,2) = [mt Jx Jy]';
% Mt(:,:,3) = [mt Jx Jy]';
% Mt(:,:,4) = [mt Jx Jy]';
% Mt(:,:,5) = [mt Jx Jy]';
% Mt(:,:,6) = [mt Jx Jy]';
% Mt(:,:,7) = [mt Jx Jy]';
% Mt(:,:,8) = [mt Jx Jy]';
% Mt(:,:,9) = [mt Jx Jy]';
% Mt(:,:,10) = [mt Jx Jy]';

% Wheel masses mw(:,:,j) = [mw_1 mw_2 mw_3 mw_4]', j = 1,2,..nt
mw = ones(4,1,nt)*1.78e3;

% Train suspension stiffness kw(:,:,j) = [kf1 kr1 kf2 kr2]', j = 1,2,..nt
kf1 = 2.976e6; kr1 = 2.976e6; kf2 = 2.976e6; kr2 = 2.976e6; 
kw(:,:,1) = [kf1 kr1 kf2 kr2]';
kw(:,:,2) = [kf1 kr1 kf2 kr2]';
% kw(:,:,3) = [kf1 kr1 kf2 kr2]';
% kw(:,:,4) = [kf1 kr1 kf2 kr2]';
% kw(:,:,5) = [kf1 kr1 kf2 kr2]';
% kw(:,:,6) = [kf1 kr1 kf2 kr2]';
% kw(:,:,7) = [kf1 kr1 kf2 kr2]';
% kw(:,:,8) = [kf1 kr1 kf2 kr2]';
% kw(:,:,9) = [kf1 kr1 kf2 kr2]';
% kw(:,:,10) = [kf1 kr1 kf2 kr2]';

% Train damping coefficient cw(:,:,j) = [cf1 cr1 cf2 cr2]', j = 1,2,..nt
cf1 = 15e3; cr1 = 15e3; cf2 = 15e3; cr2 = 15e3;
cw(:,:,1) = [cf1 cr1 cf2 cr2]';
cw(:,:,2) = [cf1 cr1 cf2 cr2]';
% cw(:,:,3) = [cf1 cr1 cf2 cr2]';
% cw(:,:,4) = [cf1 cr1 cf2 cr2]';
% cw(:,:,5) = [cf1 cr1 cf2 cr2]';
% cw(:,:,6) = [cf1 cr1 cf2 cr2]';
% cw(:,:,7) = [cf1 cr1 cf2 cr2]';
% cw(:,:,8) = [cf1 cr1 cf2 cr2]';
% cw(:,:,9) = [cf1 cr1 cf2 cr2]';
% cw(:,:,10) = [cf1 cr1 cf2 cr2]';

% Distance from train CM to front(f) and rear(r) wheel ll(:,:,j) = 
% [lf lr]', j = 1,2,..nt
lf = 5; lr = 5;
ll(:,:,1) = [lf lr]';
ll(:,:,2) = [lf lr]';
% ll(:,:,3) = [lf lr]';
% ll(:,:,4) = [lf lr]';
% ll(:,:,5) = [lf lr]';
% ll(:,:,6) = [lf lr]';
% ll(:,:,7) = [lf lr]';
% ll(:,:,8) = [lf lr]';
% ll(:,:,9) = [lf lr]';
% ll(:,:,10) = [lf lr]';

% -------------------------------------------------------------------------
% Further vehicle parameters automatically defined from user input

% Distance from train CM to left(l) and right(r) wheel lb(:,:,j) = 
% [bl br]', j = 1,2,..nt
for j=1:nt
    lb(:,:,j) = [brail/2+eCM(j) brail/2-eCM(j)]';
end

% Total train lengths
lt(:,:,1) = sum(ll,1);

% Total train widths
bt(:,:,1) = sum(lb,1);

% Include position of rear wheel in "initial" and rearrange into 3d-tensor
in = initial;
initial(:,:,1) = in;
initial(:,:,2) = in-lt;
initial(:,:,3) = in;
initial(:,:,4) = in-lt;
initial = permute(initial,[3,2,1]); % Rearranges dimensions of array to fit
                                    % notation
      
% -------------------------------------------------------------------------
% ASSEMBLY OF BRIDGE AND TRAIN SYSTEM MATRICES

% Bridge - modal system matrices (diagonal matrices n x n)
[mm,km,cm] = assemBridge(m,n,omega,zeta);  

% Train system matrices (3 x 3 x nt)
[Mt,mw,kw,cw,Kt,Ct] = assemTrain(Mt,mw,kw,cw,ll,lb,nt);

% -------------------------------------------------------------------------
% NODAL LOADS

% Gravitational acceleration
g = 9.81;

% Initialise stationary dynamic loads
Fi = zeros(nno*6,N+1);
Fi((nno+1)/2*6-4,:) = -sin(2*pi/1*tt)*2e6;

% Loads on vehicle dof's (non-gravitational loads) [u_1 phix_1 phiy_1
% u_2 phix_2 phiy_2 ... u_j phix_j phiy_j]', j = 1,2,..nt
Fv = zeros(nt*3,N+1);

% Wind load on the train is applied as a moment about phix_i.
% Wind load amplitude
Fwind = 0;
% Vertical train CM eccentricity to attacking point of wind load
eL = 0;
Fv(2,:) = sin(2*pi/1*tt)*Fwind*eL;

%% RESPONSE EXTRACTION FROM MAIN PROGRAM ----------------------------------
% MainProgram.m: Performs Newmark time integration and constructs the
% interaction vector(b) used in defining the time-varying coupled
% system matrices M, K, and C, along with the bridge load vector in each
% time step.

if ANALYSIS == 1
% -------------------------------------------------------------------------
% ANALYSIS 1: Fixed train velocity

[xb,vb,ab,xxv,xxh,Fw,xtr,vtr,atr] = MainProgram(n,nno,X,X3,T,gamma,...
    beta,dt,N,tt,initial,nt,L,ll,lb,vt,e,eh,Mt,mw,Kt,kw,Ct,cw,mm,km,cm,...
    g,Fi,Fv,S);

% OUTPUT (all response in geometric(generalised) coordinates):

% xb,vb,ab: Bridge response
% Fw: Contact forces
% xtr,vtr,atr: Vehicle response

% xxv: Vertical bridge displacement at interaction points (for animation)
% xxh: Horizontal bridge displacement at interaction points (for animation)

end

if ANALYSIS == 2
% -------------------------------------------------------------------------
% ANALYSIS 2: Train velocity

for j=1:length(vtpar)
    [xb(:,:,j),vb(:,:,j),ab(:,:,j),~,~,Fw(:,:,j),xtr(:,:,j),...
        vtr(:,:,j),atr(:,:,j)] = MainProgram(n,nno,X,X3,T,gamma,beta,dt,...
        N,tt,initial,nt,L,ll,lb,vtpar(j),e,eh,Mt,mw,Kt,kw,Ct,cw,mm,km,...
        cm,g,Fi,Fv,S);
    
    [j,length(vtpar)] % (ticker)
end

% xb,vb,ab: Bridge response for each train velocity (nno x N x
%           length(vtpar))
          
% Fw: Contact forces for each train velocity (nt*4 x (N+1) x length(vtpar))

% xtr,vtr,atr: Vehicle response for each train velocity (3 x (N+1) x 
%              length(vtpar))

end

%--------------------------------------------------------------------------
% Extract additional data and save main output
[z] = savedata(nt,ll,lb,xb,vb,ab,xtr,vtr,atr,Fw,tt);

% OUTPUT:
% z: Vehicle displacement above each suspension spring without initial
%    gravitational displacement

%% PLOT SECTION -----------------------------------------------------------

addpath('Plot functions')

% FIGURE SECTION
if ANALYSIS == 1

plotfig = 1; % plotfig == 1 => figures, plotfig ~= 1 => no figures
plotfigures(tt,xb,vb,ab,X,Fw,xxv,xxh,xtr,atr,z,plotfig,nno,n)

elseif ANALYSIS == 2

plotfig = 1;
plotfiguresPar(xtr,vtr,atr,Fw,vtpar,plotfig)

end
% -------------------------------------------------------------------------
if ANALYSIS == 1

% ANIMATION SECTION

% The animations work for bridge topology following the numerical 
% ascending order of the x-coordinates. Vehicles are only illustrated for
% bridge models with all node coordinates located on the x-axis. 
% Front: Black spring and Rear: Red spring
% Note that the horizontal translation animation is only illustrated for
% displacements above 1e-9.

animation = 1; % animation == 1 => video, animation ~= 1 => no video

% Scaling for each animation plot. Note that the horizontal translation
% animation is only illustrated for displacements above 1e(-9).

% Longitudinal animation (Bridge displacement)
scalev = 1;% Scale vertical displacements
dispvw = 1;% Display left or right wheels, dispvw = 1 => left 
           % dispvw = 3 => right
rr = 30; %Spring width scaling
scales = 0.5; %Spring height scaling

% Transverse animation (Bridge displacement)
scaleh = 1;% Scale tranverse displacements

% Cross section (Bridge displacement)
scaley = 100; % Scale transverse displacement
dispyw = 1;% Display front or rear wheels, dispyw = 1 => front 
           % dispyw = 2 => rear
dispw = 1;% Display train dispw = 1 => front train
rw = rr*28/(max(lb(:,:,dispw))+e(dispw)*2); % Spring width scaling

% Position matrix (gives each wagon position at each time step)
posf = position(N,initial(1:2,:,:),vt,tt,nt,X,T); 

% Animation
animation3d(scalev,dispvw,rr,scales,scaleh,scaley,dispyw,dispw,rw,u,X,...
    T,animation,z,xb,nno,N,nt,posf,xxv,xxh,e,dt,lb)

end