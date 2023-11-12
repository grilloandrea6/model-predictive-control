close all;
clear all;
clc;

A = [0.9752 1.4544; ... 
    -0.0327 0.9315];
B = [0.0248; 0.0327];

% Initial condition
x0 = [3; 0];

N = 10;
Q = 10*eye(2);
R = 1;

% constraints
M = [1;-1]; m = [1.75;1.75];
F = [1 0; 0 1; -1 0; 0 -1]; f = [5; 0.2; 5; 0.2];


%% compute terminal controller
% Compute LQR controller
[K,Qf,~] = dlqr(A,B,Q,R);
K = -K; % Note that matlab defines K as -K
Acl = [A+B*K]; % Closed-loop dynamics

% Compute the maximal invariant set
Xf = polytope([F;M*K],[f;m]);
while 1
        prevXf = Xf;
        [T,t] = double(Xf);      
        % Compute the pre-set
        preXf = polytope(T*Acl,t);

        Xf = intersect(Xf,preXf);
        if isequal(prevXf,Xf)
            break
        end
end

[Ff,ff] = double(Xf);

%% MPC

%optimization variables
x = sdpvar(2,N,'full');
u = sdpvar(1,N-1,'full');


% Define constraints and objective
con = [];
obj = 0;

con =(M*u(:,1) <= m);


% Define constraints and objective
con = [];
obj = 0;
for i = 1:N-1
    con = [con, x(:,i+1) == A*x(:,i) + B*u(:,i)]; % System dynamics
    if i~=1
        con = [con, F*x(:,i) <= f]; % State constraints
    end
    con = [con, M*u(:,i) <= m]; % Input constraints
    obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i); % Cost function
end
con = [con, Ff*x(:,N) <= ff]; % Terminal constraint
obj = obj + x(:,N)'*Qf*x(:,N); % Terminal weight

% Compile the matrices
ctrl = optimizer(con, obj, sdpsettings('solver','sedumi'), x(:,1), u(:,1));


%% Simulating the closed-loop system

sol.x(:,1) = x0;
i = 1;
try
    while norm(sol.x(:,end)) > 1e-3 % Simulate until convergence
        % Solve MPC problem for current state
        [uopt,infeasible] = ctrl{sol.x(:,i)};

        if infeasible == 1, error('Error in optimizer - could not solve the problem'); end

        % Extract the optimal input
        sol.u(:,i) = uopt;

        % Apply the optimal input to the system
        sol.x(:,i+1) = A*sol.x(:,i) + B*sol.u(:,i);

        i = i + 1;
    end
catch
    disp('---> Initial state is outside the feasible set <---\n');
    i
end

%% Plotting the results

figure
hold on; grid on;

o = ones(1,size(sol.x,2));

subplot(3,1,1)
hold on; grid on;
plot(sol.x(1,:),'-k','markersize',20,'linewidth',2);
plot(1:size(sol.x,2),f(1)*o,'r','linewidth',2)
plot(1:size(sol.x,2),-f(3)*o,'r','linewidth',2)
ylabel('Position')

subplot(3,1,2)
hold on; grid on;
plot(sol.x(2,:),'-k','markersize',20,'linewidth',2);
plot(1:size(sol.x,2),f(2)*o,'r','linewidth',2)
plot(1:size(sol.x,2),-f(4)*o,'r','linewidth',2)
ylabel('Velocity')

subplot(3,1,3)
o = ones(1,size(sol.u,2));
hold on; grid on;
plot(sol.u,'k','markersize',20,'linewidth',2);
plot(1:size(sol.u,2),m(1)*o,'r','linewidth',2)
plot(1:size(sol.u,2),-m(2)*o,'r','linewidth',2)
ylabel('Input')


% 
[uopt,infeasible] = ctrl{x0}
if infeasible == 1 
     error("infeasible")
end
x = x0;
figure(2)
plot(x(1),x(2))
hold on

for i = 1:50
    plot(x(1),x(2),'o')
    hold on

    x = A*x + B*uopt
    [uopt,infeasible] = ctrl{x}
    if infeasible == 1 
        error("infeasible")
    end
end
