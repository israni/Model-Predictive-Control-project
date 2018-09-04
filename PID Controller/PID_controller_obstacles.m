function [] = PID_controller_obstacles()

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

%% Load Obstacles
Nobs = 25;
Xobs = generateRandomObstacles(Nobs,TestTrack);

% Zoomed In Track
figure(2);
plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
hold on;
plot(TestTrack.br(1,:),TestTrack.br(2,:));
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
xlabel('x (m)');
ylabel('y (m)');

for i=1:Nobs
    scatter(Xobs{i}(1,1),Xobs{i}(1,2));
    scatter(Xobs{i}(2,1),Xobs{i}(2,2));
    scatter(Xobs{i}(3,1),Xobs{i}(3,2));
    scatter(Xobs{i}(4,1),Xobs{i}(4,2));
end



%% Display, Input, and Update
x = x0;
N = 20;             % Number of timesteps to integrate at a time
deltaT = 0.01; 
scale = 100;        %scale for higher accuracy
deltaT = deltaT / scale;
N = N * scale;
W = 20;
U = zeros(N,2);
disp_i = 1;

% Speed Controller (tuned for 5 m/s)
speed_K_p = 100;
brake_K_p = 100;
speed_bias = 68.6;%68.6;   %68.6;
speed_cmd = 8;

% Set Points
setpoints = TestTrack.cline;

% Loop over obstacles, finding the closest setpoint and replacing it
for i=1:Nobs
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
    sideWeight = 0.4;
    j = index1;
%     if (index1 <= index2)
%         j = index1;
%     else
%         j = index2;
%     end
    if(left)
    %if( norm(center_pt - TestTrack.bl(:,index1)) < norm(center_pt - TestTrack.br(:,index1)) )
        setpoints(:,j-2) = setpoints(:,j-2) * (1-sideWeight/2) + TestTrack.bl(:,j-2) * sideWeight/2;
        setpoints(:,j-1) = setpoints(:,j-1) * (1-sideWeight) + TestTrack.bl(:,j-1) * sideWeight;
        setpoints(:,j) = setpoints(:,j) * (1-sideWeight) + TestTrack.bl(:,j) * sideWeight;
        setpoints(:,j+1) = setpoints(:,j+1) * (1-sideWeight) + TestTrack.bl(:,j+1) * sideWeight;
        setpoints(:,j+2) = setpoints(:,j+2) * (1-sideWeight/2) + TestTrack.bl(:,j+2) * sideWeight/2;        
    else
        setpoints(:,j-2) = setpoints(:,j-2) * (1-sideWeight/2) + TestTrack.br(:,j-2) * sideWeight/2;
        setpoints(:,j-1) = setpoints(:,j-1) * (1-sideWeight) + TestTrack.br(:,j-1) * sideWeight;
        setpoints(:,j) = setpoints(:,j) * (1-sideWeight) + TestTrack.br(:,j) * sideWeight;
        setpoints(:,j+1) = setpoints(:,j+1) * (1-sideWeight) + TestTrack.br(:,j+1) * sideWeight;
        setpoints(:,j+2) = setpoints(:,j+2) * (1-sideWeight/2) + TestTrack.br(:,j+2) * sideWeight/2;
    end
    
%     A = center_pt';
%     d = norm(A-C);
%     gamma = atan2(A(2)-C(2),A(1)-C(1));
%     alpha = atan2(D(2)-C(2),D(1)-C(1));
%     theta = gamma - alpha;
    %B = C + [d*cos(alpha-theta); d*sin(alpha-theta)];
    
    
    %scatter(track_pt(1),track_pt(2),'rx');
    %scatter(C(1),C(2),'bo');
    %scatter(D(1),D(2),'bo');
    %scatter(B(1),B(2),'rx');
    scatter(center_pt(1),center_pt(2),'bo');
    %setpoints(:,index) = track_pt - (center_pt - track_pt);
    
    %setpoints(:,index1) = B;
end

plot(setpoints(1,:),setpoints(2,:));


%setpoints(1,247) = setpoints(1,246) + 5 * cos(TestTrack.theta(246));
%setpoints(2,247) = setpoints(2,246) + 5 * sin(TestTrack.theta(246));

%setpoints(3,1:246) = TestTrack.theta;
%setpoints(3,247) = setpoints(3,246);




set_i = 3;
yaw_K_p = 0.5;
dist_thresh = 5;
i = 0;
while(1)
    % Speed P Controller
    speed_error = (speed_cmd - x(1,2));
    if(speed_error > 0)
        F_x = speed_error * speed_K_p + speed_bias;
    else
        F_x = speed_error * brake_K_p + speed_bias;
    end
    
    % Yaw P Controller
    y_err = setpoints(2,set_i) - x(1,3);
    x_err = setpoints(1,set_i) - x(1,1);
    set_pt_dist = norm([y_err x_err]);
    if(set_pt_dist < dist_thresh)
        set_i = set_i + 1;
        if(set_i == 247)
            break;
        end
    end
    %yaw_setpt = 0.7*setpoints(3,set_i) + 0.3*atan2(y_err,x_err);%atan2(y_err,x_err); 
    yaw_setpt = atan2(y_err,x_err);
    yaw_error = yaw_setpt - x(1,5);
    d_f = yaw_error * yaw_K_p;
    
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
time = length(x_tot) * 0.01 / scale;
disp(strcat('Time:   ',num2str(time,6),'(s)'));

%% De-scale for testing
x_tot_ = x_tot(1:scale:(end-1),:);
U_tot_ = U_tot(1:scale:end,:);

%% Check against real checker


if(checkTrajectory([x_tot_(1:end,1),x_tot_(1:end,3)],U_tot_))
    disp('ode1 simulator says you hit something!');
    save('manual_failure_ode1.mat','x_tot_','U_tot_');
else
    disp('ode1 simulator says you passed! Good Job!');  
    save('manual_success_ode1.mat','x_tot_','U_tot_');
end

% [Y]=forwardIntegrateControlInput(U_tot,x0);

% if(checkTrajectory([Y(:,1),Y(:,3)],U_tot))
%     disp('How much have you had to drink? You hit something!');
% else
%     disp('Who are you Dale Earnhardt Jr? Nice Driving!');
%     save('manual_success.mat','x_tot','U_tot');
% end

