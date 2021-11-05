clear all;

load map1
% make a random array of 100 x 100
%map = randi(2,size(map)) - 1
%map = ones(100) 
%map = zeros(100)
map = rot90(diag(ones(50,1), 5)) % <-- why does this go straight through the line!?
bug = Bug2(map)
bug.plot()
goal = [50,30];
start = [20, 10];
bug.query(start, goal, 'animate');