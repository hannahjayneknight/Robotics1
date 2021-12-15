clear all;
syms alpha beta gamma real;

Rz = [cos(alpha) -sin(alpha) 0;... %Use ... to go to the next row. It looks better that way
     sin(alpha) cos(alpha) 0;...
     0 0 1];
 
Ry = [cos(beta) 0 sin(beta);...
    0 1 0;...
    -sin(beta) 0 cos(beta)];

Rx = [1 0 0;...
    0 cos(gamma) -sin(gamma);...
    0 sin(gamma) cos(gamma)];

R = simplify(Rz*Ry*Rx)
Rlatex = latex(R)

%%
%Test for properties of a rotation matrix
alpha = 2*pi*rand
beta = 2*pi*rand
gamma = 2*pi*rand

RR = eval(R)
%Determinant of R
dRR = det(RR);
disp(['Determinant of R: ', num2str(dRR)]);
%Orthonormality of R
R1 = RR(:,1); R2 = RR(:,2); R3 = RR(:,3);

disp(['Norm of R1: ',num2str(norm(R1)), ', norm of R2: ',num2str(norm(R2)), ', norm of R3: ',num2str(norm(R3))]);
disp(['R1 dot R2: ', num2str(R1'*R2/(norm(R1)*norm(R2)))]);
disp(['R1 dot R3: ', num2str(R1'*R3/(norm(R1)*norm(R3)))]);
disp(['R2 dot R3: ', num2str(R2'*R3/(norm(R2)*norm(R3)))]);


