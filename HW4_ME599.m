%HW4 ME599
%% Problem 1
A=[0 0.310625; 0 0];
B=[0;1];
x0=[-1;0];
% 1.1
dt=0.01;

n=10/dt+1;
x_traj=zeros(2,n);
x_traj(:,1)=x0;


for k=2:n
    x_traj(:,k)=(eye(2)+A*dt)*x_traj(:,k-1)+dt*B*-0.3175*sin((pi/10)*dt*(k-1)-pi/2);
end

%1.2
Ndec=32; %11 steps of state decision variables, 10 steps of input decision variables

%1.3
n_steps=11;
Ad=(eye(2)+A*dt);
Bd=dt*B;
A=Ad;
B=Bd;
Aeq=zeros(22,32);
beq=zeros(22,1);
Aeq(1,1)=1;
Aeq(2,2)=1;
xnom=[-1;0];
for i=1:n_steps-1
        Aeq(i*size(Ad,1)+1:i*size(Ad,1)+2,i*size(Ad,1)-1:i*size(Ad,1))=Ad;
        Aeq(i*size(Ad,1)+1:i*size(Ad,1)+2,i*size(Ad,1)+1:i*size(Ad,1)+2)=-eye(2);
        Aeq(i*size(Ad,1)+1:i*size(Ad,1)+2,i+22)=Bd;
end
beq(1:2,1)=[-1.25;0]-[-1;0];
    

%1.4
Aineq=zeros(64,32);
bineq=zeros(22,1);
n_states=n_steps*2;
for i=1:1:16
    k=0;

    Aineq(i*size(Ad)-1:i*size(Ad),i*size(Ad)-1:i*size(Ad))=eye(2);

    Aineq(i*size(Ad)+Ndec-1:i*size(Ad)+Ndec,i*size(Ad)-1:i*size(Ad))=-eye(2);
end
k=0;
bineq(1:n_states)=0.5;
bineq(n_states+1:Ndec)=10-(-0.3175*sin((pi/10)*dt*(k)-pi/2));
bineq(Ndec+1:Ndec+n_states)=0.5;
bineq(Ndec+n_states+1:2*Ndec)=10+(-0.3175*sin((pi/10)*dt*(k)-pi/2));
% for i=1:2:64
%     if i<=n_states
%         bineq(i:i+1)=0.5;
%     else
%         bineq(i:i+1)=
%     end
%     
%     if i<=Ndec+n_states
%         bineq(i:i+1)=0.5;
%     else
%         bineq(i:i+1)=10+(-0.3175*sin((pi/10)*dt*(k)-pi/2));
%     end
% end

%1.5
tspan=0:dt:10;
x=zeros(2,length(tspan));
dx=zeros(2,length(tspan));
u=zeros(1,length(tspan)-1);
du=zeros(1,length(tspan)-1);
x(:,1)=[-1.25;0];
dx(:,1)=[-0.25;0];
Q=[100 0;0 100];
%Q=100*eye(2);
lb=[-.5;-0.5];
ub=[0.5;0.5];
H = zeros(Ndec);
c = zeros(Ndec, 1 );
for i=1:11
H((i-1)*2+1:i*2,(i-1)*2+1:i*2)=Q;
end
%H(21:22,21:22)=Q;
bineq_curr=zeros(64,1);
beq_curr=zeros(22,1);
init_state=[-0.25;0];
for k = 0:999

    beq_curr(1:2)=dx(:,k+1); 
    bineq_curr(1:n_states)=0.5;
    bineq_curr(Ndec+1:Ndec+n_states)=0.5;    
    bineq_curr(n_states+1:Ndec)=10-(-0.3175*sin((pi/10)*dt*(k+1)-pi/2));
    bineq_curr(Ndec+n_states+1:2*Ndec)=10+(-0.3175*sin((pi/10)*dt*(k+1)-pi/2));
    out=quadprog(H,c,Aineq,bineq_curr,Aeq,beq_curr);
    du(k+1)=out(n_states+1);
    
    dx(:,k+2)=A*dx(:,k+1)+B*du(k+1);
    x(:,k+2)=A*x(:,k+1)+B*(du(k+1)+(-0.3175*sin((pi/10)*dt*(k)-pi/2)));
    %init_state=dx(:,k+2);

end 

%% Problem 2
vars = load('simple_nn_vars.mat');
n_epoch = 2000;
rate = [0.04, 0.2];
test_intv = 100;

output_nn = simple_nn(vars, n_epoch, rate, test_intv);

%% Problem 3
close all;
vars = load('simple_cnn_vars.mat');
n_epoch = 700;
rate = 0.01;
p = 2;
test_intv = 50;

output_cnn = simple_cnn(vars, n_epoch, rate, p, test_intv);
