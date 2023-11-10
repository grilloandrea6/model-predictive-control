clear all
close all
clc
A = [.7115 -.4345; ...
    .4345 .8853];
B = [.2173; .0573];
C = [0 1];

%% design of the observer using augmented system
Bd = [0;0];
Cd = 1;
augTest = [A-eye(2) Bd; C Cd];

Aaug = [A zeros(2, 1); 0 0 1];
Baug = [B;Bd];
Caug = [C Cd];

L = -place (Aaug',Caug',[.5 .6 .7])';


%% find a steady state
r = 1;

xs = sdpvar(2,1,'full');
us = sdpvar(1,1,'full');

obj = us'*us;

const = (-3 <= us) + ( us <= 3) + (xs == A*xs + B*us) + (r == C*xs);

diagnostics = solvesdp(const,obj);

if diagnostics.problem == 0
   % Good! 
elseif diagnostics.problem == 1
    throw(MException('','Infeasible'));
else
    throw(MException('','Something else happened'));
end


us = double(us);
xs = double(xs);


%% MPC controller 
% no terminal set
% terminal cost deltaXn' P deltaXn


%horizon length
N    = 5;

% Step cost function
Q    = eye(2);
R    = 1;

% P solution of P-A'PA=Q
P = dlyap(A,Q);

nsteps = 50;

x_hist = zeros(2,nsteps);
x_hat_hist = zeros(2,nsteps);
d_hat_hist = zeros(1,nsteps);
u_hist = zeros(1,nsteps);

x_hist(:,1) = [1;2];
x_hat_hist(:,1) = [3;0];
d_hat_hist(1) = 0;
d = .2;
u_hist(1) = 0;

u = sdpvar(1,N);

for i= 1:nsteps
    x = x_hist(:,i);
    
    

    % Define constraints and objective
    con = u == u_hist(i);
    obj = 0;
    for i = 1:N-1
        x = A*x + B*u(i);
        con = [con, x(:,i+1) == A*(x(:,i)-xs) + B*(u(:,i)-us)]; % System dynamics
        con = [con, u(i) >= -3, u(i) <= 3]; % Input constraints
        obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(i)-us)'*R*(u(i)-us); % Cost function
    end
    
    obj = obj + x(:,N)'*P*x(:,N); % Terminal weight
    
    % Compile the matrices
    ctrl = optimizer(con, obj, sdpsettings('solver','sedumi'), x(:,1), u(:,1));
    

    
end








% x0 = [1;2;.2];
% x= zeros(3,150);
% y = zeros(1,150);
% x(:,1) = x0;
% y(1,:) = Caug * x0;



%for i = 2:100
%    x(:,i+1) = L*x(:,i);
%    y(i,:) = Caug * x(:,i);
%end
