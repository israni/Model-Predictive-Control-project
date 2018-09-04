%% double_integrator.m with our system

global gridN
gridN = 20;

tic
% Minimize the simulation time
time_min = @(z) z(1);
% The initial parameter guess; 1 second, gridN positions, gridN velocities,
% gridN accelerations

z0 = [1;zeros(8*gridN,1)];
% set velocity
for i=3:6:117
    z0(i) = 1;
end
% set x position
for i=2:6:116
    z0(i) = ((i-2)/6 + 1) / 20;
end
% set y position
for i=4:6:118
    z0(i) = 0;
end

% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations between -10 and 30
lb = [0;    ones(gridN * 6, 1) * -Inf;  ones(gridN, 1) * -0.5; ones(gridN, 1) * -10000];
ub = [Inf;  ones(gridN * 6, 1) * Inf;   ones(gridN, 1) *  0.5; ones(gridN, 1) *   5000];

lb(4:6:118) = 0;    %lb on y to be 0
% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'iter', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp');%,...
                       %'SpecifyConstraintGradient',true);
% Solve for the best simulation time + control input
[optimal,FVAL,EXITFLAG,OUTPUT,LAMBDA,GRAD,HESSIAN] = fmincon(time_min, ...
              z0, A, b, Aeq, Beq, lb, ub, ...
              @double_integrator_constraints_us, options);

% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;
% Get the state + accelerations (control inputs) out of the vector
x_positions = optimal(2:6:116); %1 + gridN*6);
y_positions = optimal(4:6:118);
d_f = optimal(122:2:160);
F_x = optimal(123:2:161);

% Make the plots
figure();
plot(x_positions,y_positions);
xlim([-2 2]);
ylim([-2 2]);
title('Trajectory');

figure();
plot(times,d_f);
title('Steering input');

figure();
plot(times,F_x);
title('Throttle');

disp(sprintf('Finished in %f seconds', toc));