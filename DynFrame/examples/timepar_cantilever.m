% File: timepar_cantilever.m
%   Input data file data used in the dynamic analysis.

% time history
dt = 0.01;
N  = 200;
t0 = 0;
tt = linspace(t0,dt*N,N+1);

% Rayleigh damping
a = 2.231; 
b = 0.000532; 

% external load
Fd = zeros(ndof,N+1);
% Sine load 
Fd(45,:) = -sin(2*pi/(0.243)*tt)*2e5;

% initial conditions
x0 = zeros(ndof,1);
v0 = zeros(ndof,1);

% Weighting parameters
gamma = 0.5;
beta = 0.25;

% Modal damping ratio
zeta = 0.05;