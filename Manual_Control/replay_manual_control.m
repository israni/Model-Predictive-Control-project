function [] = replay_manual_control(filename)
%% x0
x0 = [287, 5, -176, 0, 2, 0]; 

%% Load Map
load('TestTrack.mat');
load(filename);
if( strcmp(filename,'manual_success.mat') ||...
    strcmp(filename,'manual_success_ode1.mat') ||...
    strcmp(filename,'manual_failure_ode1.mat') )
    U_tot = U_tot_;
    x_tot = x_tot_;
    ROB599_ControlsProject_part1_input = U_tot;
end
disp('Forward Integrating Inputs now!...');
if(~exist('sol_1'))
    sol_1 = forwardIntegrateControlInput(ROB599_ControlsProject_part1_input);
end
disp('Checking Trajectory now!...');
failed = checkTrajectory([sol_1(:,1),sol_1(:,3)],ROB599_ControlsProject_part1_input);





%%

%%


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
W = 40;    % Size of Viewing Window About Car
sol_1;

for i=1:N:length(sol_1)
    h1 = scatter(sol_1(i,1),sol_1(i,3));
    xlim([(sol_1(i,1)-W) (sol_1(i,1)+W)]);
    ylim([(sol_1(i,3)-W) (sol_1(i,3)+W)]);
    v = sol_1(i,2)
    xoff = cos(sol_1(i,5))/20;
    yoff = sin(sol_1(i,5))/20;
    h2 = annotation('arrow', [0.5175 - xoff, 0.52 + xoff], [0.5175 - yoff, 0.52 + yoff] );
    
    
    pause(0.01);
    delete(h1);
    delete(h2);
end
h1 = scatter(sol_1(i,1),sol_1(i,3));
h2 = annotation('arrow', [0.5175 - xoff, 0.52 + xoff], [0.5175 - yoff, 0.52 + yoff] );
hold off;

% Full Window Track
figure(1);
plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
hold on;
plot(TestTrack.br(1,:),TestTrack.br(2,:));
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
plot(sol_1(:,1),sol_1(:,3));
xlabel('x (m)');
ylabel('y (m)');
legend('left barrier','right barrier','center line','trajectory');
%time = length(x_tot) * 0.01
disp(failed);


%% Display States of ode1 vs ode45
figure();
plot(sol_1(:,1));
hold on;
plot(x_tot(:,1));
hold off;
title('x');
ylabel('x');
legend('ode45','ode1');

figure();
plot(sol_1(:,2));
hold on;
plot(x_tot(:,2));
hold off;
title('u');
ylabel('u');
legend('ode45','ode1');

figure();
plot(sol_1(:,3));
hold on;
plot(x_tot(:,3));
hold off;
title('y');
ylabel('y');
legend('ode45','ode1');

figure();
plot(sol_1(:,4));
hold on;
plot(x_tot(:,4));
hold off;
title('v');
ylabel('v');
legend('ode45','ode1');

figure();
plot(sol_1(:,5));
hold on;
plot(x_tot(:,5));
hold off;
title('\psi');
ylabel('\psi');
legend('ode45','ode1');

figure();
plot(sol_1(:,6));
hold on;
plot(x_tot(:,6));
hold off;
title('r');
ylabel('r');
legend('ode45','ode1');

figure();
plot(U_tot(:,1));
title('d_f');

figure();
plot(U_tot(:,2));
title('F_x');



end

