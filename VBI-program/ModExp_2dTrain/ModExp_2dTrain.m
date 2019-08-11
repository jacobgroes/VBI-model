%*************************************************************************
% File: ModExp_2dTrain.m
%   Driver for linear-elastic dynamic analysis of vehicle-bridge 
%   interaction. The bridge is constructed from modal expansion of a 
%   linear-elastic 3d-beam-structure in a seperate software. 
%   2d-mass-spring-damper vehicles are constructed in the input section.
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

% Unit prefix of input mode-shapes (x[m])*10^y and rotations 
% (theta[rad])*10^y. If in millimeter, y = 3 - if in kilometer, y = -3 - 
% if in meter, y = 0.
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
zeta = 0.05;

% Natural angular frequencies
omega(:,1) = f(1:n)*2*pi;

% Number of nodes in the FE-model
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
initial = [0]';

% Number of trains 
nt = length(initial);

% Train velocity
vt = 100; % Fixed velocity for - ANALYSIS 1
vtpar = [30:5:100]; % Velocities for parametric study - ANALYSIS 2

% Horizontal train eccentricity to bridge shear center
e = 0;

% Vertical train CM eccentricity to bridge shear center
eh = 0;

% Train mass properties Mt(:,:,j) = [mt Jy]', j = 1,2,..nt
mt = 2e5; % Train mass
Jy = 2e6; % Train mass moment of inertia about the y-axis
Mt(:,:,1) = [mt Jy]';
%Mt(:,:,2) = [mt Jy]';
%Mt(:,:,3) = [mt Jy]';

% Wheel masses mw(:,:,j) = [mw_1 mw_2]', j = 1,2,..nt
mw = ones(2,1,nt)*200;

% Train suspension stiffness kw(:,:,j) = [kf kr]', j = 1,2,..nt
kf = 2e6; kr = 2e6;
kw(:,:,1) = [kf kr]';
%kw(:,:,2) = [kf kr]';
%kw(:,:,3) = [kf kr]';

% Train damping coefficient cw(:,:,j) = [cf cr]', j = 1,2,..nt
cf = 15e3; cr = 15e3;
cw(:,:,1) = [cf cr]';
%cw(:,:,2) = [cf cr]';
%cw(:,:,3) = [cf cr]';

% Distance from train CG to front(f) and rear(r) wheel ll(:,:,j) =
% [lf lr]', j = 1,2,..nt
lf = 6; lr = 6;
l(:,:,1) = [lf lr]';
%l(:,:,2) = [lf lr]';
%l(:,:,3) = [lf lr]';

% Train lengths
lt(:,:,1) = sum(l,1);

% Include rear wheel in "initial" and rearrange into 3d-tensor
initial(:,:,1) = initial;
initial(:,:,2) = initial-lt;
initial = permute(initial,[3,2,1]); % Rearranges dimensions of array to fit
                                    % notation

% -------------------------------------------------------------------------
% ASSEMBLY OF BRIDGE AND TRAIN SYSTEM MATRICES

% Bridge - modal system matrices (diagonal matrices n x n)
[mm,km,cm] = assemBridge(m,n,omega,zeta);  

% Train system matrices (2 x 2 x nt)
[Mt,mw,kw,cw,Kt,Ct] = assemTrain(Mt,mw,kw,cw,l,nt);

% -------------------------------------------------------------------------
% NODAL LOADS

% Gravitational acceleration
g = 9.81;

% Dynamic bridge loads
Fi = zeros(nno*6,N+1);
Fi((nno+1)/2*6-4,:) = -sin(2*pi/1*tt)*2e3;

%% RESPONSE EXTRACTION FROM MAIN PROGRAM ----------------------------------
% Main program: Performs Newmark time integration and constructs the
% interaction vector(b) used in defining the time-varying coupled
% system matrices M, K, and C, along with the bridge load vector in each
% time step.

if ANALYSIS == 1
% -------------------------------------------------------------------------
% ANALYSIS 1: Fixed train velocity

[xb,vb,ab,xxv,xxh,Fw,xtr,vtr,atr] = MainProgram(n,nno,X,X3,T,gamma,...
    beta,dt,N,tt,initial,nt,L,l,vt,e,eh,Mt,mw,Kt,kw,Ct,cw,mm,km,cm,g,Fi,S);

% OUTPUT (all response in geometric(generalised) coordinates):

% xb,vb,ab: Bridge response
% Fw: Contact forces 
% xtr,vtr,atr: Vehicle response

% xxv: Vertical bridge displacement at interaction points (animation)
% xxh: Horizontal bridge displacement at interaction points (animation)

end

if ANALYSIS == 2
% -------------------------------------------------------------------------
% ANALYSIS 2: Train velocity

for j=1:length(vtpar)
    [xb(:,:,j),vb(:,:,j),ab(:,:,j),~,~,Fw(:,:,j),xtr(:,:,j),...
        vtr(:,:,j),atr(:,:,j)] = MainProgram(n,nno,X,X3,T,gamma,beta,...
        dt,N,tt,initial,nt,L,l,vtpar(j),e,eh,Mt,mw,...
        Kt,kw,Ct,cw,mm,km,cm,g,Fi,S);
        
    [j,length(vtpar)] % (ticker)
end

% xb,vb,ab: Bridge response for each train velocity (nno x N x
%           length(vtpar))
          
% Fw: Contact forces for each train velocity (nt*2 x (N+1) x length(vtpar))

% xtr,vtr,atr: Vehicle response for each train velocity (2 x (N+1) x 
%              length(vtpar))

end

%--------------------------------------------------------------------------
% Extract additional data and save main output
[z] = savedata(nt,l,xb,vb,ab,xtr,vtr,atr,Fw,tt);

% Output:
% z: Vehicle displacement above each suspension spring without initial
%    gravitational displacement

%% PLOT SECTION -----------------------------------------------------------

addpath('Plot functions')

% FIGURE SECTION
if ANALYSIS == 1
    
plotfig = 1; % plotfig == 1 => figures, plotfig ~= 1 => no figures
plotfigures(tt,xb,vb,ab,X,Fw,xxv,xxh,xtr,atr,z,plotfig,nno,n,nt)

elseif ANALYSIS == 2
    
plotfig = 1;
plotfiguresPar(xtr,vtr,atr,Fw,vtpar,plotfig)

end
% -------------------------------------------------------------------------

if ANALYSIS == 1
% ANIMATION SECTION

% The animations work for bridge topology following the numerical 
% ascending order of the x-coordinates. Vehicles are only illustrated for
% bridge models with all node coordinates located on the x-axis. Note that
% the horizontal translation animation is only illustrated for 
% displacements above 1e-9.

animation = 1; % animation == 1 => video, animation ~= 1 => no video

% Scaling for each animation plot. 

% Longitudinal animation (Bridge displacement)
scalev = 1;% Scale vertical displacements
rr = 30; %Spring width scaling
scales = 0.5; %Spring height scaling

% Transverse animation (Bridge displacement)
scaleh = 1;% Scale tranverse displacements

% Cross section (Bridge displacement)
scaley = 100; % Scale transverse displacement
dispyw = 1;% Display front or rear wheels, dispyw = 1 => front, 
           %                               dispyw = 2 => rear
dispw = 1;% Display train dispw = 1 => front train
rw = 100000/(1+e*100); % Spring width scaling

% Position matrix (gives each wagon position at each time step)
posf = position(N,initial,vt,tt,nt,X,T);

% Animation
animation2d(scalev,rr,scales,scaleh,scaley,dispyw,dispw,rw,u,X,T,...
    animation,z,xb,nno,N,nt,posf,xxv,xxh,e,dt)

end
