function [x,v,a,t] = timestep2(omega0,zeta,x0,v0,t0,dt,n,f)
%
%  TIMESTEP2
%    Exact time-stepping for linear, damped, forced oscillator. The
%    normalized load is piecewise linear over intervals of length dt.
%    Created for the course 41214: Dynamics of Structure: Theory and
%    Analysis at DTU 
%  INPUT
%    Physical parameters
%      omega0   : undamped natural angular frequency.
%      zeta     : damping ratio (fraction of critical).
%    Initial conditions
%      x0       : initial displacement.
%      v0       : initial velocity.
%      t0       : starting time.
%    Stepping parameters
%      dt       : size of time step.
%      n        : number of time steps.
%    Load history
%      f        : vector of load amplitudes, optional.
%
%  OUTPUT
%    Histories
%      x        : response history, x(1:n+1).
%      v        : velocity history, v(1:n+1).
%    Time
%      t        : discrete times,   t(1:n+1).
%
%  VERSION
%    20.01.1998
%    Dept. of Structural Engineering and Materials
%    Technical University of Denmark
%-------------------------------------------------------------------

% Set load vector
if nargin == 7
  f = zeros(1,n+1);
end
if length(f) < n+1
  f = [f, zeros(1,n+1-length(f))];
end

% Compute coefficient matrices
omegad = omega0*sqrt(1-zeta^2);
alpha  = omegad*dt;
beta   = zeta*omega0*dt;
A(1,1) = alpha*cos(alpha)+beta*sin(alpha);
A(1,2) = sin(alpha);
A(2,1) = -(alpha^2+beta^2)*sin(alpha);
A(2,2) = alpha*cos(alpha)-beta*sin(alpha);
A      = A * exp(-beta)/alpha;
B(1,1) = 2*alpha*beta*(1-exp(-beta)*cos(alpha))/(alpha^2+beta^2) ...
         + (alpha^2-beta^2)*exp(-beta)*sin(alpha)/(alpha^2+beta^2) ...
         - exp(-beta)*(alpha*cos(alpha)+beta*sin(alpha));
B(1,2) =-2*alpha*beta*(1-exp(-beta)*cos(alpha))/(alpha^2+beta^2) ...
         - (alpha^2-beta^2)*exp(-beta)*sin(alpha)/(alpha^2+beta^2) ...
         + alpha;
B(2,1) = (alpha^2+beta^2+beta)*exp(-beta)*sin(alpha) ...
         - alpha*(1-exp(-beta)*cos(alpha));
B(2,2) = alpha*(1-exp(-beta)*cos(alpha)) ...
         - beta*exp(-beta)*sin(alpha);
B      =  B / (omega0^2*alpha);

% Set initial conditions
y(1,1) = x0;
y(2,1) = dt*v0;

% Compute time history
for i = 1:n
  y(:,i+1) = A * y(:,i) + B * f(i:i+1)';
end

% Set output
x = y(1,:);
v = y(2,:)/dt;
a = f - 2*zeta*omega0*v - omega0^2*x;
t = ones(1,n+1)*t0 + [0:n]*dt;
