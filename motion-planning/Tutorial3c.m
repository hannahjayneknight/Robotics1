clear all
close all

%%% RRT example 1 %%%
goal = [0,0,0];
start = [0,2,0];
vehicle = Bicycle('steermax', 1.2); % create an agent
rrt = RRT(vehicle, 'goal', goal, 'range', 5);
rrt.plan() % create navigation tree
p = rrt.query(start, goal); % animate path from this start location
figure;
rrt.plot(p)
plot_vehicle(p, 'box', 'size', [1 1.5], 'fill', 'r', 'alpha', 0.1);

return

%%% RRT example 2 %%%
load road; % load the map
car = Bicycle('steermax', 0.5); % create the agent
rrt = RRT(car, road, 'npoints', 300, 'root', [50 22 0], 'simtime', 4);
rrt.plan()
p = rrt.query([40 45 0], [50 22 0]);
figure;
rrt.plot(p)
plot_vehicle(p, 'box', 'size', [6 9], 'fill', 'r', 'alpha', 0.1);

