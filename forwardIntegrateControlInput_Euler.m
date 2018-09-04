function [x]=forwardIntegrateControlInput_Euler(U,x0,deltaT)
%function [Y]=forwardIntegrateControlInput_Euler(U,x0)
%
%Given a set of inputs and an initial condition, returns the vehicles
%trajectory. If no initial condition is specified the default for the track
%is used.
%
% INPUTS:
%   U           an N-by-2 vector of inputs, where the first column is the
%               steering input in radians, and the second column is the 
%               longitudinal force in Newtons.
%   
%   x0          a 1-by-6 vector of the initial state of the vehicle.
%
% OUTPUTS:
%   x           an (N+1)-by-6 vector where each column is the trajectory of the
%               state of the vehicle (the first row is x0)
%
% Written by: Matthew Porter
% Created: 13 Nov 2017
% Modified: 16 Nov 2017
% 
% Modified: 5 Dec 2017 by Matthew Romano for Euler

%if initial condition not given use default
if nargin<2
    x0=[287,5,-176,0,2,0];
end

% Note: we are now assuming inputs at 0.01 steps and doing simple ode1
%deltaT = 0.01;
[N,~] = size(U); 

%constants
W=13720;
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=2921;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
      
%slip angle functions in degrees
a_f=@(d_f,x) rad2deg(d_f-atan2(x(4)+a*x(6),x(2)));
a_r=@(x) rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=@(d_f,x) (1-Ey)*(a_f(d_f,x)+Shy)+(Ey/By)*atan(By*(a_f(d_f,x)+Shy));
phi_yr=@(x) (1-Ey)*(a_r(x)+Shy)+(Ey/By)*atan(By*(a_r(x)+Shy));

F_yf=@(d_f,x) Dy*sin(Cy*atan(By*phi_yf(d_f,x)))+Svy;
F_yr=@(x) Dy*sin(Cy*atan(By*phi_yr(x)))+Svy;

%vehicle dynamics
df=@(d_f,F_x,x) [x(2)*cos(x(5))-x(4)*sin(x(5));...
      (-f*W+Nw*F_x-F_yf(d_f,x)*sin(d_f))/m+x(4)*x(6);...
      x(2)*sin(x(5))+x(4)*cos(x(5));...
      (F_yf(d_f,x)*cos(d_f)+F_yr(x))/m-x(2)*x(6);...
      x(6);...
      (F_yf(d_f,x)*a*cos(d_f)-F_yr(x)*b)/Iz];      

%Solve for trajectory     
x = zeros(N+1,6);
x(1,:) = x0;
for i=1:N
    x(i+1,:) = x(i,:) + deltaT * df(U(i,1),U(i,2),x(i,:))'; 
end

end


