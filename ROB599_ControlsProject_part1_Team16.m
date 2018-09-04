function [x_tot_, U_tot_] = ROB599_ControlsProject_part1_Team16(TestTrack)
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
%% Load Map
load('TestTrack.mat');

% Zoomed In Track
figure();
plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
hold on;
plot(TestTrack.br(1,:),TestTrack.br(2,:));
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
xlabel('x (m)');
ylabel('y (m)');


%% Display, Input, and Update
x = x0;
N = 20;             % Number of timesteps to integrate at a time
deltaT = 0.01;
scale = 100;        %scale for higher accuracy
deltaT = deltaT / scale;
N = N * scale;
W = 40;    % Size of Viewing Window About Car
U = zeros(N,2);
U_tot = zeros(N,2);
x_tot = zeros(N,6);
i = 0;
disp_i = 1;

% Speed Controller (tuned for 5 m/s)
speed_K_p = 150;
brake_K_p = 300;
speed_bias = 68.6;%68.6;   %68.6;
super_slow_speed_cmd = 5;
slow_speed_cmd = 10;
high_speed_cmd = 18;
super_high_speed_cmd = 22;
super_super_high_speed_cmd = 22;

% Yaw Controller
setpoints = TestTrack.cline;
setpoints(1,247) = setpoints(1,246) + 5 * cos(TestTrack.theta(246));
setpoints(2,247) = setpoints(2,246) + 5 * sin(TestTrack.theta(246));
setpoints(3,1:246) = TestTrack.theta;
setpoints(3,247) = setpoints(3,246);
curvature = diff(setpoints(3,:));
setpoints(4,1:247) = high_speed_cmd;
setpoints(4,248) = high_speed_cmd;

slow_spots = find(abs(curvature) > 0.1);
for j=1:3
    i=1;
    while (i <= length(slow_spots) )
        if(isempty(find(slow_spots == (slow_spots(i) - 1))))
            slow_spots = [slow_spots(1:i-1),slow_spots(i)-1,slow_spots(i:end)]; 
        end
        i = i +1;
    end
end
setpoints(4,slow_spots) = slow_speed_cmd;

super_slow_spots = find(abs(curvature) > 0.2);
for j=1:3
    i=1;
    while (i <= length(super_slow_spots) )
        if(isempty(find(super_slow_spots == (super_slow_spots(i) - 1))))
            super_slow_spots = [super_slow_spots(1:i-1),super_slow_spots(i)-1,super_slow_spots(i:end)]; 
        end
        i = i +1;
    end
end
setpoints(4,super_slow_spots) = super_slow_speed_cmd;

setpoints(4,205:220) = super_high_speed_cmd;
setpoints(4,221:end) = super_super_high_speed_cmd;

%% End Condition
end_m = (TestTrack.bl(2,246)-TestTrack.br(2,246))/(TestTrack.bl(1,246)-TestTrack.br(1,246));
end_b =  TestTrack.bl(2,246) - end_m*TestTrack.bl(1,246);


set_i = 3;
yaw_K_p = 0.55;
yaw_K_d = 0.05;
dist_thresh = 8;
i = 0;
while(1)
    % Speed P Controller
    speed_error = (setpoints(4,set_i+1) - x(1,2));
    if(speed_error > 0)
        F_x = speed_error * speed_K_p + speed_bias;
    else
        F_x = speed_error * brake_K_p + speed_bias;
    end
    
    % Yaw P Controller
    if(set_i > 215)
        y_err = setpoints(2,247) - x(1,3);
        x_err = setpoints(1,247) - x(1,1);
    else
        y_err = setpoints(2,set_i) - x(1,3);
        x_err = setpoints(1,set_i) - x(1,1);
    end
    set_pt_dist = norm([y_err x_err]);
    if(set_pt_dist < dist_thresh)
        set_i = set_i + 1;
        %if(set_i == 247)
        %    break;
        %end
    end
    
    % If we cross the finish line break
    if((end_m * x(1,1) + end_b) <= x(1,3) )
       break; 
    end
    
    yaw_setpt = 0.7*setpoints(3,set_i) + 0.3*atan2(y_err,x_err);%atan2(y_err,x_err); 
    yaw_error = yaw_setpt - x(1,5);
    d_f = yaw_error * yaw_K_p + x(1,6) * yaw_K_d;
    
    U = input_dynamics(d_f, F_x, U, N, scale);
   
%     if(mod(i,150) == 149)
%         break;
%     end
    
    if(i == 0)
        %tic;
        U_tot = U;
    else
        U_tot = [U_tot; U];
    end
    
    [Y]=forwardIntegrateControlInput_Euler(U,x,deltaT);
    if(i == 0)
        x_tot = Y;
    else
        Y(1,:) = [];
        x_tot = [x_tot; Y];
    end
    x = Y(end,:);
    
    if(mod(i,disp_i) == 0)
        h1 = scatter(x(1,1),x(1,3));
        xlim([(x(1,1)-W) (x(1,1)+W)]);
        ylim([(x(1,3)-W) (x(1,3)+W)]);
        
        v = x(1,2)
        
        xoff = cos(x(1,5))/20;
        yoff = sin(x(1,5))/20;
        h3 = annotation('arrow', [0.5175 - xoff, 0.52 + xoff], [0.5175 - yoff, 0.52 + yoff] );
        
        drawnow
        delete(h1);
        delete(h3);
    end
    i = i+1;
end

h1 = scatter(x(1,1),x(1,3));
time = length(x_tot) * 0.01 / scale;
disp(strcat('Time:   ',num2str(time,6),'(s)'));

%% De-scale for testing
x_tot_ = x_tot(1:scale:(end-1),:);
U_tot_ = U_tot(1:scale:end,:);

%% Check against real checker


if(checkTrajectory([x_tot_(1:end,1),x_tot_(1:end,3)],U_tot_))
    disp('ode1 simulator says you hit something!');
    save('failure_ode1.mat','x_tot_','U_tot_');
else
    disp('ode1 simulator says you passed! Good Job!');  
    save('success_ode1.mat','x_tot_','U_tot_');
end

% [Y]=forwardIntegrateControlInput(U_tot,x0);

% if(checkTrajectory([Y(:,1),Y(:,3)],U_tot))
%     disp('How much have you had to drink? You hit something!');
% else
%     disp('Who are you Dale Earnhardt Jr? Nice Driving!');
%     save('manual_success.mat','x_tot','U_tot');
% end

