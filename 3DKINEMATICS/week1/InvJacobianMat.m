clear all;
syms l1 l2 l3 theta1 theta2 real;

Ttip = [l1*cos(theta1) + l2*sin(theta1) + l3*cos(theta1)*cos(theta2);...
    l1*sin(theta1) - l2*cos(theta1) + l3*sin(theta1)*cos(theta2);...
    l3*sin(theta2)];

%%          Obtain the Jacobian matrix and the pseudo inverse 

J = jacobian(Ttip,[theta1,theta2]); %Jacobian matrix 

Jinv = simplify(pinv(J))


%%
T = 0.02;%Sampling step size

M11 = [];
M21 = [];
M12 = [];
M22 = [];
M13 = [];
M23 = [];
for i = 1:12
    for j = 1:12
    ang = [i*0.1;j*0.1]; 
    M11(i,j) = vpa(subs(Jinv(1,1),[theta1,theta2,l1,l2,l3],[ang',1,1,1]));
    M12(i,j) = vpa(subs(Jinv(1,2),[theta1,theta2,l1,l2,l3],[ang',1,1,1]));
    M13(i,j) = vpa(subs(Jinv(1,3),[theta1,theta2,l1,l2,l3],[ang',1,1,1]));
    
    M21(i,j) = vpa(subs(Jinv(2,1),[theta1,theta2,l1,l2,l3],[ang',1,1,1]));
    M22(i,j) = vpa(subs(Jinv(2,2),[theta1,theta2,l1,l2,l3],[ang',1,1,1]));
    M23(i,j) = vpa(subs(Jinv(2,3),[theta1,theta2,l1,l2,l3],[ang',1,1,1]));
    end
end

%%
xx = [0.1:0.1:1.2]';
yy = [0.1:0.1:1.2]';
figure;
subplot(2,3,1);
surf(xx,yy,M11);colorbar;
xlabel('\theta_1 [rad]','fontsize',16);
ylabel('\theta_2 [rad]','fontsize',16);
zlabel('Jinv_{11} ','fontsize',16);
%axis([0.1 1.2 0.1 1.2 0 1]);

subplot(2,3,2);
surf(xx,yy,M12);colorbar;
xlabel('\theta_1 [rad]','fontsize',16);
ylabel('\theta_2 [rad]','fontsize',16);
zlabel('Jinv_{12} ','fontsize',16);

subplot(2,3,3);
surf(xx,yy,M13);colorbar;
xlabel('\theta_1 [rad]','fontsize',16);
ylabel('\theta_2 [rad]','fontsize',16);
zlabel('Jinv_{13} ','fontsize',16);

subplot(2,3,4);
surf(xx,yy,M21);colorbar;
xlabel('\theta_1 [rad]','fontsize',16);
ylabel('\theta_2 [rad]','fontsize',16);
zlabel('Jinv_{21} ','fontsize',16);

subplot(2,3,5);
surf(xx,yy,M22);colorbar;
xlabel('\theta_1 [rad]','fontsize',16);
ylabel('\theta_2 [rad]','fontsize',16);
zlabel('Jinv_{22} ','fontsize',16);

subplot(2,3,6);
surf(xx,yy,M23);colorbar;
xlabel('\theta_1 [rad]','fontsize',16);
ylabel('\theta_2 [rad]','fontsize',16);
zlabel('Jinv_{23} ','fontsize',16);