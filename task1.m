% Task 1

%% Init
load('TestTrack.mat');

%% Run PID on track
[X_ode1, U] = ROB599_ControlsProject_part1_Team16(TestTrack);

%% Forward Integrate Using ode45
[X] = forwardIntegrateControlInput(U);

%% Check this Trajectory
failed = checkTrajectory_informative(X,U);

if(failed)
    disp('Failure');
else
    disp('Success');
end

%% Pad input if necessary
U = [U; zeros(100,2)];
X = forwardIntegrateControlInput(U);
while(checkTrajectory(X,U) == 1)
    X = X(1:end-1,:);
    U = U(1:end-1,:);
end

%% Check again
failed = checkTrajectory_informative(X,U);

if(failed)
    disp('Failure');
else
    disp('Success');
end
