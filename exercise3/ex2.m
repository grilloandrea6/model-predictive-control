clear all
close all
clc
alpha = pi/6;
beta = 0.8;

A = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)] * beta;
B = [.5 .5]';

c3 = cos(pi/3);
s3 = sin(pi/3);

H = [c3 s3; -c3 -s3; s3 -c3; -s3 c3];
h = [2 1 2 5]';

G = [1 -1]';
g = [.5 .5]';


X = Polyhedron(H,h);



cont = 0;


while 1
    cont = cont + 1;

    preX = projection(Polyhedron([X.A*A X.A*B; zeros(2,2) G],  [X.b;g]),[1:2]);
    
    nextX = Polyhedron([preX.A;X.A],[preX.b;X.b]);
    
    if X == nextX
        Cinf = X;
        break;
    end


    X = nextX;
    
end
figure(1)
hold on


plot(Polyhedron(H,h),'alpha',0.5,'color','blue');
plot(Cinf,'alpha',0.7,'color','green')



Q = eye(2);
R = 1;
[K,~,~] = dlqr(A,B,Q,R);
K = -K; % to make the system matrix A+BK instead of A-BK

% now I want to execute the first exercise code using 
% A+BK instead of A as the matrix

X = Polyhedron([H;G*K],[h;g]);

cont = 0;
%G = G*K;



while 1
    cont = cont + 1;


    preX = Polyhedron(X.A * (A+B*K),X.b);
    
    nextX = Polyhedron([preX.A;X.A],[preX.b;X.b]);
    

    if X == nextX
        Oinf = X;
        break;
    end


    X = nextX;
    
end
disp(cont)
plot(Oinf)

legend('X','Cinf','Oinf')




%% Cinf e il meglio che qualsiasi controllore possa fare
%% Oinf e cio che LQR puo controllare, che e un controllore lineare, quindi e piu restrittivo, piu piccolo
%vedi slide 3 week3
