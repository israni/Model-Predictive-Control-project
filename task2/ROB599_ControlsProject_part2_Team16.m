function [sol_2] = ROB599_ControlsProject_part2_Team16(TestTrack,Xobs)
%% Simple PID_controller on waypoints and speed

%% Initial Condition
%   x0          a 1-by-6 vector of the initial state of the vehicle.
%       x = x-position       (m)
%       u = forward velocity (m/s)
%       y = y-position       (m)
%       v = lateral velocity (m/s)
%     psi = yaw              (rad)
%       r = yaw rate         (rad/s)

x0 = [287, 5, -176, 0, 2, 0]; 

%% Load Obstacles
Nobs = length(Xobs);

%% Display, Input, and Update
x = x0;
N = 20;             % Number of timesteps to integrate at a time
deltaT = 0.01; 
scale = 100;        %scale for higher accuracy
deltaT = deltaT / scale;
N = N * scale;
U = zeros(N,2);

% Speed Controller (tuned for 5 m/s)
speed_K_p = 100;
brake_K_p = 100;
speed_bias = 68.6;%68.6;   %68.6;
speed_cmd = 4;

% Set Points
setpoints = TestTrack.cline;
new_setpoints = zeros(Nobs,2,246);

% Loop over obstacles, finding the closest setpoint and replacing it
for i=1:Nobs
    new_setpoints(i,:,:) = setpoints;
    
    obstacle = Xobs{i};
    %track_pt = (obstacle(1,:) + obstacle(2,:) ) / 2;
    
    %% Find the closest point
    center_pt = (obstacle(1,:) + obstacle(2,:) + obstacle(3,:) + obstacle(4,:) ) / 4;
    differences = setpoints - center_pt' * ones(1,246);
    value = zeros(1,246);
    for j=1:246
        value(j) = norm(differences(:,j));
    end
    
    % First and second closest points (index1 is the closest pt still)
    [~,index1] = min(value);
    if( norm(center_pt - setpoints(:,index1+1)') < norm(center_pt - setpoints(:,index1-1)') ) %untranspose setpoints for newer Matlab versions (>2015)
        index2 = index1 + 1;
        angle = atan2(center_pt(2) - setpoints(2,index1),center_pt(1) - setpoints(1,index1));
        left = (angle < TestTrack.theta(index1) );
    else
        index2 = index1 - 1;
        angle = atan2(center_pt(2) - setpoints(2,index2),center_pt(1) - setpoints(1,index2));
        left = (angle < TestTrack.theta(index2) ); 
    end
    
    % Is it to the left or the right of the closest pt?'
    sideWeight = 0.5;
    if (index1 <= index2)
        j = index1;
    else
        j = index2;
    end
    if(left)
    %if( norm(center_pt - TestTrack.bl(:,index1)) < norm(center_pt - TestTrack.br(:,index1)) )
        new_setpoints(i,:,j-2) = setpoints(:,j-2) * (1-sideWeight/2) + TestTrack.bl(:,j-2) * sideWeight/2;
        new_setpoints(i,:,j-1) = setpoints(:,j-1) * (1-sideWeight) + TestTrack.bl(:,j-1) * sideWeight;
        new_setpoints(i,:,j) = setpoints(:,j) * (1-sideWeight) + TestTrack.bl(:,j) * sideWeight;
        new_setpoints(i,:,j+1) = setpoints(:,j+1) * (1-sideWeight) + TestTrack.bl(:,j+1) * sideWeight;
        new_setpoints(i,:,j+2) = setpoints(:,j+2) * (1-sideWeight/2) + TestTrack.bl(:,j+2) * sideWeight/2;        
    else
        new_setpoints(i,:,j-2) = setpoints(:,j-2) * (1-sideWeight/2) + TestTrack.br(:,j-2) * sideWeight/2;
        new_setpoints(i,:,j-1) = setpoints(:,j-1) * (1-sideWeight) + TestTrack.br(:,j-1) * sideWeight;
        new_setpoints(i,:,j) = setpoints(:,j) * (1-sideWeight) + TestTrack.br(:,j) * sideWeight;
        new_setpoints(i,:,j+1) = setpoints(:,j+1) * (1-sideWeight) + TestTrack.br(:,j+1) * sideWeight;
        new_setpoints(i,:,j+2) = setpoints(:,j+2) * (1-sideWeight/2) + TestTrack.br(:,j+2) * sideWeight/2;
    end    
end

%% Parse through differing setpoints from obstacles
% will simple average just work? 
% average on non standard points will though
final_setpoints = setpoints;
for j=1:246
    diff_pts = [];
    setpoints(:,j);
    for i=1:Nobs
        % if differing point save it
        if(new_setpoints(i,:,j) ~= setpoints(:,j) )
            diff_pts = [diff_pts; new_setpoints(i,:,j) ];
        end
    end
    % Take the average of the diff_pts if non-empty
    if(~isempty(diff_pts))
        final_setpoints(:,j) = [mean(diff_pts(:,1)); mean(diff_pts(:,2))]; 
    end
end

final_setpoints(1,247) = setpoints(1,246) + 10 * cos(TestTrack.theta(246));
final_setpoints(2,247) = setpoints(2,246) + 10 * sin(TestTrack.theta(246));

%% End Condition
end_m = (TestTrack.bl(2,246)-TestTrack.br(2,246))/(TestTrack.bl(1,246)-TestTrack.br(1,246));
end_b =  TestTrack.bl(2,246) - end_m*TestTrack.bl(1,246);

set_i = 3;
yaw_K_p = 0.5;
dist_thresh = 5;
i = 0;
U_tot = zeros(2458000,2);
x_tot = zeros(2458001,6);
while(1)
    % Speed P Controller
    speed_error = (speed_cmd - x(1,2));
    if(speed_error > 0)
        F_x = speed_error * speed_K_p + speed_bias;
    else
        F_x = speed_error * brake_K_p + speed_bias;
    end
    
    % Yaw P Controller
    y_err = final_setpoints(2,set_i) - x(1,3);
    x_err = final_setpoints(1,set_i) - x(1,1);
    set_pt_dist = norm([y_err x_err]);
    if(set_pt_dist < dist_thresh)
        set_i = set_i + 1;
    end
    
    % If we cross the finish line break
    if((end_m * x(1,1) + end_b) <= x(1,3) )
       break; 
    end
    
    %yaw_setpt = 0.7*setpoints(3,set_i) + 0.3*atan2(y_err,x_err);%atan2(y_err,x_err); 
    yaw_setpt = atan2(y_err,x_err);
    yaw_error = yaw_setpt - x(1,5);
    d_f = yaw_error * yaw_K_p;
    
    U = input_dynamics_(d_f, F_x, U, N, scale);
    U_tot(2000*i+1:2000*(i+1),:) = U;
    %if(i == 0)
    %    U_tot = U;
    %else
    %    U_tot = [U_tot; U];
    %end
    
    [Y]=forwardIntegrateControlInput_Euler_(U,x,deltaT);
    if (i == 0) 
        x_tot(1:2001,:) = Y;
    else 
        Y(1,:) = [];
        x_tot(2000*i+2:2000*(i+1)+1,:) = Y;
    end
    
    x = Y(end,:);
    
    i = i+1;
    elapsed=toc;
    if elapsed>300
        break;
    end
end

%% Prepare Output
sol_2 = U_tot(1:scale:end,:);
sol_2 = [sol_2; zeros(100,2)];   %pad zeros to cross goal for sure
end

function [U] = input_dynamics_(d_f, F_x, U_last, N, scale)
d_f_rate = 0.003;
F_x_rate = 100;

% Don't allow input out of range
if(d_f >= 0.5)
    d_f = 0.5;
end
if(d_f <= -0.5)
    d_f = -0.5;
end
if(F_x >= 5000)
    F_x = 5000;
end
if(F_x <= -10000)
    F_x = -10000;
end



% Don't allow input to change instantaneously
U = zeros(N,2);

% d_f dynamics at most full left to full right in 1 sec (0.01 per iter)
d_f_last = U_last(end,1);
d_f_error = d_f_last - d_f;

if( abs(d_f_error) <= d_f_rate)
   U(:,1) = d_f * ones(N,1); 
else
    if (d_f_error > 0)
        d_f_step = -1*d_f_rate;
    else
        d_f_step = d_f_rate;
    end
    for i=1:scale:N
        U(i:(i+scale-1),1) = (d_f_last + d_f_step) * ones(scale,1);
        d_f_last = U(i,1);
        d_f_error = d_f_error + d_f_step;
        if(abs(d_f_error) <= d_f_rate)
            U(i+scale:end,1) = d_f *ones(N-(i+scale)+1,1);
            break;
        end
    end
end

% F_x dynamics at most 10,000 per 0.1 sec == 1000 per second
F_x_last = U_last(end,2);
F_x_error = F_x_last - F_x;

if( abs(F_x_error) <= F_x_rate)
   U(:,2) = F_x * ones(N,1); 
else
    if (F_x_error > 0)
        F_x_step = -1*F_x_rate;
    else
        F_x_step = F_x_rate;
    end    
    for i=1:scale:N
        U(i:(i+scale-1),2) = (F_x_last + F_x_step) * ones(scale,1);
        F_x_last = U(i,2);
        F_x_error = F_x_error + F_x_step;
        if(abs(F_x_error) <= F_x_rate)
            U(i+scale:end,2) = F_x * ones(N-(i+scale)+1,1);
            break;
        end
    end
end


end

function [x]=forwardIntegrateControlInput_Euler_(U,x0,deltaT)
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

