%% Load Map
close all;
clear all;
clc;
addpath(genpath('./'));

disp('Planning ...');
map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);
%plot_path(map);

%% Calculate Path from each start to each Goal
start  = {[0  -5   0], [10  -5  0], [5  -5  0]};       %Map 1
goal   = {[5  17   5], [1   14  5], [9  20  5]};       %Map 1
% start  = {[0   -5  0]};                     %Map 1
% stop   = {[10  20  6]};

% stop = {[0  -5 0]};                     %Map 2
% start  = {[10 30 5]};                   %Map 2
% start  = {[19 3 5]};
% stop   = {[1  3 5]};
nquad   = length(start);
flength = length(goal);
dist = zeros(nquad,flength);
cost_path = cell(nquad,flength);

for qn = 1:nquad
    for l = 1:flength
        cost_path{qn,l} = Astar(map, start{qn}, goal{l});
        dist(qn,l) = sum(sqrt(sum(diff(cost_path{qn,l}).^2,2)))^2;          % Creates a vector of the absolute distance between points
    end
end
[assignment,cost] = munkres(dist);
stop = cell(flength);
path = cell(nquad,flength);
for l = 1:flength
    stop{l} = goal{assignment(l)};
    path{l} = cost_path{l,assignment(l)};
end
%plot_path(map,path{qn});
%% Resolve the Assignment of start points and goals


%% Additional init script
init_script;
clc
disp('Initialization Finished!');

% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true);                 % with visualization










