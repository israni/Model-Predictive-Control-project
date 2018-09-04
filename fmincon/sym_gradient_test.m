%% Gradient of our System

% State
syms x u y v psi r
syms x_ u_ y_ v_ psi_ r_
% 
syms d_f F_x
%
syms sim_time
gridN = 20

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

%%
%slip angle functions in degrees
a_f = (d_f-atan2(v+a*r,u)) / pi * 180;
a_r = (-atan2((v-b*r),u))  / pi * 180;

%% Nonlinear Tire Dynamics
phi_yf= (1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr= (1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_yf= Dy*sin(Cy*atan(By*phi_yf))+Svy;
F_yr= Dy*sin(Cy*atan(By*phi_yr))+Svy;

%% vehicle dynamics
df=  [u * cos(psi)-v*sin(psi);...
      (-f*W+Nw*F_x-F_yf*sin(d_f))/m+v*r;...
      u*sin(psi)+v*cos(psi);...
      (F_yf*cos(d_f)+F_yr)/m-u*r;...
      r;...
      (F_yf*a*cos(d_f)-F_yr*b)/Iz];
  
%%
df_jacob = jacobian(df,[x u y v psi r d_f F_x]);

%% Euler Step
delta_time = sim_time / gridN;

ceq_eul_sym = [x_ y_ u_ v_ psi_ r_]' - ([x y u v psi r]' + delta_time * df);

%% Take Gradient

grad_ceq_eul_sym = jacobian(ceq_eul_sym,[sim_time x y u v psi r d_f F_x x_ y_ u_ v_ psi_ r_]);
%% Evaluate to test
subs(grad_ceq_eul_sym,[sim_time x y u v psi r d_f F_x x_ y_ u_ v_ psi_ r_],[]);





  
  