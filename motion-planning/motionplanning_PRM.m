clear all
close all

%%% PRM example 1 %%%
load map1 % load map
goal = [50,30]; % goal point
start = [20, 10]; % start point
prm = PRM(map); % create navigation object
prm.plan('npoints', 150) % create roadmaps
prm.query(start, goal) % animate path from this start location
figure;
prm.plot();

return

%%% PRM example 2 %%%
load house % load map
goal = place.kitchen; % goal point
start = place.br3; % start point
prm = PRM(house); % create navigation object
prm.plan('npoints', 150) % create roadmaps
prm.query(start, goal) % animate path from this start location
figure;
prm.plot();

