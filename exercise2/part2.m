
clear all;
close all;
clc;

n = 50;
x = logspace(log10(1e-3),0,n);
y = zeros(1,n);
% Problem:
%  min  0.5 * z' * prob.H * z + prob.q' * z
%  s.t. prob.G * z <= prob.d

%% Choose problem parameters
dim   = 2; % Number of optimization variables
speed = 'fast'; % set to 'fast' for fast convergence, and 'slow' to see what's going on
seed  = ceil(100*rand); % Set to any integer to choose the randomly generated problem

[prob,opt] = setupEx2(dim, speed, seed);


for i = 1:n-1
    opt.mu = x(i);
    y(i) = ex2(prob,opt,dim);
end
hold off
figure(3)
plot(x(1:end-1),y(1:end-1))

