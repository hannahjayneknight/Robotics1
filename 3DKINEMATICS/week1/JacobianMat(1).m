%-----------Exercise-3.3-------------------
clear all;
syms l0 l1 l2 l3 theta1 theta2 theta3 real

%     ai alphai    di thetai
DH = [0     0      0 theta1;...   % Joint 1
      l1 sym(pi)/2 l2 theta2;...   % Joint 2
      l3 0  0 theta3];     % Joint 3  

 %Now we can separate them to DH parameter columns
a = DH(:,1);
alp = DH(:,2);
d = DH(:,3);
th = DH(:,4);

TN = size(DH,1);
%Finding each transformation matrix using Khalil and Dombre variation method (eq 38)
for i = 1:TN
    T(i).A =  [cos(th(i)) -sin(th(i)) 0 a(i);...
               sin(th(i))*cos(alp(i)) cos(th(i))*cos(alp(i)) -sin(alp(i)) -sin(alp(i))*d(i);...
               sin(th(i))*sin(alp(i)) cos(th(i))*sin(alp(i)) cos(alp(i)) cos(alp(i))*d(i);...
               0 0 0 1];
end
%transformtion matrix for each link
Link1 = T(1).A
Link2 = T(2).A
Link3 = T(3).A
%Now we construct the homogeneous transformation matrix for the tip
Ttip = T(1).A; 
for i = 2:TN
    Ttip = Ttip*T(i).A;
end

Ttip = simplify(Ttip) %simplified transformation matrix from tip to base

%%           Review for tutorial 
%---------------   Example 1  -------------------

%Now we can substitute joint angle values to find the tip position and
%orientation

tipPos_exp = Ttip([1:3],4) %tip position as a function

J = jacobian(tipPos_exp,[theta1,theta2]) %Jacobian matrix 

% BTtip = vpa(subs(Ttip,[theta1,theta2,theta3,l0,l1,l2],[pi/6,pi/6,pi/6,0.3,0.2,0.3])); %substitute values
% 
% tipPos = BTtip([1:3],4) %tip position relative to the base
% 
% tipOrient = acosd(double(BTtip(1,1))) %angle between the x axes of last link and first link
