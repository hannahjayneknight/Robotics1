%{

A new frame assignment convention involves two screws to transform frame F1 to F0. The first
screw translates the origin of F1 frame by a distance a along the x-axis of F1 while rotating
around the x-axis of F1 by an angle α. Let this intermediate frame be Fi. The second screw
translates the origin of Fi frame by a distance b along the y-axis of F0 while rotating around the
y-axis of F0 by an angle θ. What is the compound transformation matrix from F1 to F0?

%}

syms a b c alpha theta real;
P = [a*cos(alpha)+b*cos(alpha + theta); a*sin(alpha)+b*sin(alpha + theta); c]; % The given position vector
J = jacobian(P,[alpha theta]); % Take the Jacobian with respect to alpha and theta joint angles
F = [2 1 3]'; % Force vector
tau = J'*F;