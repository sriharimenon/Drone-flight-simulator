close all;
clear all;
clc;
addpath(genpath('./'));

disp('Hello! Choose from the following maps to simulate flight in:');
disp('1. Through a hole and above a bench 2.Obstacle sprint 3. Along a corridor');

choice=input('enter map:');


%% Plan path
disp('Planning ...');
if choice==1
    map = load_map('map0.txt', 0.2, 0.5, 0.2);
    start = {[0.0  -4.9 0.2]};
    stop  = {[8.0  18.0 3.0]};
end

if choice==2
    map = load_map('map3.txt', 0.2, 0.5, 0.2);
    start = {[1.5 0 0]};
    stop  = {[20 5 6]};
end

if choice==3
    map = load_map('map2.txt', 0.2, 0.5, 0.2);
    start = {[0.0 -4.9 0.2]};
    stop  = {[10.0 30.0 5.0]};
end

nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
