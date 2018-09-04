function []= driving_simulator()
%% Visual Simulator with User Input


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

% Full Window Track
% figure(1);
% plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
% hold on;
% plot(TestTrack.br(1,:),TestTrack.br(2,:));
% plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
% xlabel('x (m)');
% ylabel('y (m)');

% Zoomed In Track
figure(2);
plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
hold on;
plot(TestTrack.br(1,:),TestTrack.br(2,:));
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
xlabel('x (m)');
ylabel('y (m)');


%% Display, User Input, and Update
x = x0;
N = 20;     % Number of timesteps to integrate at a time
deltaT = 0.01;
scale = 100; %scale for higher accuracy
deltaT = deltaT / scale;
N = N * scale;
W = 40;    % Size of Viewing Window About Car
U = zeros(N,2);
U_tot = zeros(N,2);
x_tot = zeros(N,6);
%x_tot = x0;
%x_tot = repmat(x0,20,1);
i = 0;
len_last = 0;
x_last = x0;

d_f = 0;
F_x = 0;

% Speed Controller (tuned for 5 m/s)
speed_K_p = 100;
speed_bias = 100;%68.6;

% Yaw Controller
setpoints = TestTrack.cline;
setpoints(1,247) = setpoints(1,246) + 10 * cos(TestTrack.theta(246));
setpoints(2,247) = setpoints(2,246) + 10 * sin(TestTrack.theta(246));
set_i = 3;
yaw_K_p = 0.5;
dist_thresh = 5;

while(1)
    [d_f, F_x, Esc] = get_360_input();
    
    % Speed P Controller
    speed_error = (5 - x(1,2));
    F_x = speed_error * speed_K_p + speed_bias;
    
    % Yaw P Controller
    y_err = setpoints(2,set_i) - x(1,3);
    x_err = setpoints(1,set_i) - x(1,1);
    set_pt_dist = norm([y_err x_err]);
    if(set_pt_dist < dist_thresh)
        set_i = set_i + 1;
        if(set_i == 248)
            break;
        end
    end
    yaw_setpt = atan2(y_err,x_err); 
    yaw_error = yaw_setpt - x(1,5);
    d_f = yaw_error * yaw_K_p;
    
    U = input_dynamics(d_f, F_x, U, N, scale);
    %U(:,1) = d_f * ones(N,1);
    %U(:,2) = F_x * ones(N,1);
    
    if(Esc)
        break;
    end
   
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

    %figure(2)
    
    
    if(mod(i,50) == 1)
        h1 = scatter(x(1,1),x(1,3));
        xlim([(x(1,1)-W) (x(1,1)+W)]);
        ylim([(x(1,3)-W) (x(1,3)+W)]);

        %dim = [.2 .5 .3 .3];
        %str = strcat('Velocity = ',num2str(x(1,2)));
        %h2 = annotation('textbox',dim,'String',str,'FitBoxToText','on');
        %v = x(1,2)

        xoff = cos(x(1,5))/20;
        yoff = sin(x(1,5))/20;
        h3 = annotation('arrow', [0.5175 - xoff, 0.52 + xoff], [0.5175 - yoff, 0.52 + yoff] );
        %h3.Color = [(x(1,2)/50) 0 0];
        drawnow
        %pause(0.0000001);
        %pause(N/100/scale - toc - 0.004);%0.1 - toc);
        %tic
        delete(h1);
        %delete(h2);
        delete(h3);
    end
    
%     if(mod(i,10) == 0)
%         figure(1)
%         scatter(x(1,1),x(1,3));
%     end

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