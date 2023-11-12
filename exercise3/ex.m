clear all
close all
clc
alpha = pi/6;
beta = 0.8;

A = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)] * beta;

c3 = cos(pi/3);
s3 = sin(pi/3);

H = [c3 s3; -c3 -s3; s3 -c3; -s3 c3];
h = [2 1 2 5]';



X = Polyhedron(H,h);



cont = 0;


while 1
    cont = cont + 1;

    preX = Polyhedron(X.A * A,X.b);
    
    nextX = Polyhedron([preX.A;X.A],[preX.b;X.b]);
    
    if X == nextX
        Oinf = X;
        break;
    end


    X = nextX;
    
end
disp(cont)
figure(1)
hold on


Polyhedron(H,h).plot;
Oinf.plot


x = [-1;1];

for i = 1:100
    plot(x(1),x(2),"ko")
    x = A*x;
end

