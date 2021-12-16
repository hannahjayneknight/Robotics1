%{

A new frame assignment convention involves two screws to transform frame F1 to F0. The first
screw translates the origin of F1 frame by a distance a along the x-axis of F1 while rotating
around the x-axis of F1 by an angle α. Let this intermediate frame be Fi. The second screw
translates the origin of Fi frame by a distance b along the y-axis of F0 while rotating around the
y-axis of F0 by an angle θ. What is the compound transformation matrix from F1 to F0?

%}

clear all; syms alpha theta a b;
Rx = [  1 0 0; 
        0 cos(alpha) -sin(alpha); 
        0 sin(alpha) cos(alpha) ]; % Rotation matrix around x axis

Px = [  a;
        0;
        0   ]; % Origin translation vector along the x axis

Ry = [  cos(theta) 0 sin(theta); 
        0 1 0; 
        -sin(theta) 0 cos(theta)]; % Rotation matrix around the y axis

Py = [  0;
        b;
        0   ]; % Origin translation vector along the y axis

S1 = [  Rx Px;
        0 0 0 1 ]; % First screw for rotation and translation on the x axis 
S2 = [  Ry Py;
        0 0 0 1 ]; % Second screw for rotation and translation on the y axis

Tc = simplify(S2*S1); % correct because we implement screw-1 first and screw-2 second
% Wrong Tw = simplify(S1*S2)



