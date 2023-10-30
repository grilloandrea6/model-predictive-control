clear all;
close all
clc
format long

A = [4/3 -2/3; 1 0];
B = [1; 0];
C = [-2/3 1];


Q = C'*C + 0.001*eye(2);
R = .001;
Pf = Q;

%horizon
N = 8;

% calculate K so that then u = K x
% slide 2-30

K=[]; H =[];
Hi = Q;

for i = N-1:-1:0
    Ki = -(R+B'*Hi*B)\B'*Hi*A;

    Hi = Q + Ki'*R*Ki + (A+B*Ki)' * Hi*(A+B*Ki);

    K = [ Ki' K ];
    H = [ Hi' H ];
end

% to check stability, calculate eigenvalues of the resulting matrix
% we will always use K(1), so the A resultant matrix will be A+BK
Atilde = A+B*K(:,1)';

eig(Atilde); % discrete system, should be |lambdai| < 1
% with N >= 7 it's enough, with 7 however one is really near 1, better to
% use 8


%% first I do the 3rd part, so then I simulate and calculate the cost altogheter
[Knew,S,CLP] = dlqr(A,B,Q,R);

%Knew are the gains for the infinite horizon LQR



% part 2 -> simulation
xi = [10;10];
x=xi;
u=[];


M = 50; %M steps of simulation
VfiniteHorizon = 0; 
VinfiniteHorizon = 0;

for i = 1:1:M
    VfiniteHorizon = VfiniteHorizon + xi'*Q*xi + xi'*K(:,1)*R*K(:,1)'*xi; % calculate 

    ui = K(:,1)'*xi;
    xi = A*xi + B*ui;
    u = [u ui];
    x = [x xi];
end
plot(x(1,:),x(2,:),'o-')


for i = 1:1:M
    VinfiniteHorizon = VinfiniteHorizon + xi'*Q*xi + xi'*Knew'*R*Knew*xi; % calculate 

    ui = Knew*xi;
    xi = A*xi + B*ui;
    u = [u ui];
    x = [x xi];
end

figure(2)
plot(x(1,:),x(2,:),'o-')




% part 2 -> simulation


disp(VfiniteHorizon)
disp(VinfiniteHorizon)