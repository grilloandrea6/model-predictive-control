clear all
close all
clc
A = [.7115 -.4345; ...
    .4345 .8853];
B = [.2173; .0573];
C = [0 1];

Bd = [0;0];
Cd = 1;
augTest = [A-eye(2) Bd; C Cd];

Aaug = [A zeros(2, 1); 0 0 1];
Baug = [B;Bd];
Caug = [C Cd];

L = -place (Aaug',Caug',[.1 .2 .3])';

x0 = [1;2;.2];
x= zeros(3,150);
y = zeros(1,150);
x(:,1) = x0;
y(1,:) = Caug * x0;
for i = 2:100
    x(:,i+1) = L*x(:,i);
    y(i,:) = Caug * x(:,i);
end
