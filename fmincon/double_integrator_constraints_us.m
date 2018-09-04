function [ c, ceq] = double_integrator_constraints_us( z )
%function [ c, ceq, GC, GCeq] = double_integrator_constraints_us( z )
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
      
    
    global gridN
    % No nonlinear inequality constraint needed
    c = []; % This will be the track constraints later... (hard)
    
    
    % Calculate the timestep
    sim_time = z(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    X = zeros(gridN,6);
    U = zeros(gridN,2);
    
    X = [z(2:6:116),z(3:6:117),z(4:6:118),z(5:6:119),z(6:6:120),z(7:6:121)]; 
    U = [z(122:2:160), z(123:2:161)];
    
    % Constrain initial position and velocity to be zero
    %x0=[287,5,-176,0,2,0]';
    x0=[0,1,0,0,0,0]';
    xf=[1,1,0,0,0,0]';
    ceq = [z(2:7) - x0];
    
    for i = 1 : gridN - 1
        X_df = df(U(i,1),U(i,2),X(i,:));
        %ceq = [ceq ; subs(ceq_eul_sym,[x u y v psi r ...x_ u_], [X(i,1) X(i,2) ...]) ];
        %subs(grad_ceq_eul_sym,[x u y v psi r ...x_ u_], [X(i,1) X(i,2) ... U(i,1) U(i,2) X(i+1,1) ...])
        ceq= [ceq ; X(i+1,:)' - (X(i,:)' + delta_time * X_df)];
    end 
    
         
        
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; X(end,1) - xf(1); X(end,3)-xf(3)];
    
    
    %% Gradients of Constraints
    GCeq = zeros(length(z),length(ceq));
    GC = zeros(length(z),length(c));
    [gceq_states,gc_eq_constr] = size(GCeq);
    
    % ceq = [z(2:7) - x0];
    GCeq(2,1) = 1;
    GCeq(3,2) = 1;
    GCeq(4,3) = 1;
    GCeq(5,4) = 1;
    GCeq(6,5) = 1;
    GCeq(7,6) = 1;
    
    % ceq = [ceq ; X(end,1) - xf(1); X(end,3)-xf(3)];
    GCeq(116,121) = 1;
    GCeq(118,122) = 1;
    
end