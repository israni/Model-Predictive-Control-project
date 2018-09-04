function [failed] = checkTrajectory_informative(x,U,Xobs)
%checkTrajectory_informative A better and more informative version
%   of the checkTrajectory function (with plots!)

% Input
    % x : FULL State (Nx6)      (0.01s discretizations)
             % Official Traj via forwardIntegrateControllerIinput.m
    % U : Input      (Nx2)      (0.01s discretizations)
    % Xobs:          (1xNobs)
if nargin<3
    failed = checkTrajectory([x(:,1),x(:,3)],U);
else
    failed = checkTrajectory([x(:,1),x(:,3)],U,Xobs);
end

%% x0
x0 = [287, 5, -176, 0, 2, 0]; 

%% Load Map
load('TestTrack.mat');

% Plot Zoomed In Track
figure();
plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
hold on;
plot(TestTrack.br(1,:),TestTrack.br(2,:));
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
xlabel('x (m)');
ylabel('y (m)');

%% Plot Obstacles
if nargin <3
    disp('no obstacles');
else
    for i=1:length(Xobs)
        scatter(Xobs{i}(1,1),Xobs{i}(1,2));
        scatter(Xobs{i}(2,1),Xobs{i}(2,2));
        scatter(Xobs{i}(3,1),Xobs{i}(3,2));
        scatter(Xobs{i}(4,1),Xobs{i}(4,2));
    end
end

%% Display, User Input, and Update
N = 20;     % Number of timesteps to integrate at a time
W = 40;    % Size of Viewing Window About Car

for i=1:N:length(x)
    h1 = scatter(x(i,1),x(i,3));
    xlim([(x(i,1)-W) (x(i,1)+W)]);
    ylim([(x(i,3)-W) (x(i,3)+W)]);
    v = x(i,2);
    xoff = cos(x(i,5))/20;
    yoff = sin(x(i,5))/20;
    h2 = annotation('arrow', [0.5175 - xoff, 0.52 + xoff], [0.5175 - yoff, 0.52 + yoff] );
    drawnow; 
    delete(h1);
    delete(h2);
end
h1 = scatter(x(i,1),x(i,3));
h2 = annotation('arrow', [0.5175 - xoff, 0.52 + xoff], [0.5175 - yoff, 0.52 + yoff] );
hold off;

% Full Window Track
figure();
plot(TestTrack.bl(1,:),TestTrack.bl(2,:));
hold on;
plot(TestTrack.br(1,:),TestTrack.br(2,:));
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));
plot(x(:,1),x(:,3));
xlabel('x (m)');
ylabel('y (m)');
legend('left barrier','right barrier','center line','trajectory');

end

