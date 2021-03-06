%% Microrobot Swarm Aggregation using Medial Axes for Path Planning
% ========================================================================
% Author: Li Huang 
% Email:lihuang.mech@gmail.com
% ========================================================================


clear;
close all;
clc

% rng(18)

% Def number of robots
NumRob = 1024; 

% Animation ON/OFF switch
animation_switch = 1;
cost_visual = 1;
logcost = 0;
% save frame
save_frame = 0;

% Process Display ON/OFF
proc_disp = 0;

% Choose an Algorithm
alg = 2;   % 1 for Benchmark heuristics
          % 2 for Divide-and-conquer standard
          % 3 for Divide-and-conquer V2
          % 4 for Divide-and-conquer V1
          % 5 for Divide-and-conquer V21
          % 6 for Divide-and-conquer V11

% If bolus_region == 0, use uniform distribution          
bolus_region = 0; %          
          
% Functions
funct =4;
%1: map processing only
%2: Global control
%3: Map evaluation (under construction)
%4: Human control (joystick required)


% Specify vascular network maps
mapname = {'figT','figStdMap','figVB','maze1','maze2','maze3', 'figVein'};

% target range
target_range = {15,15,15,15,15,15,15};
% Specify map scaling
scale = {[40 70],[40 70],[60 70],[40 70],[40 70],[40,70],[40, 70]};

% Goal locations for each map
goalloc = {[242, 273], [354,356],[260,122],[372,164],[307 567],[351,180], [160, 511]};% 
% STD [157, 214], [354,356]
% map1203 [211, 295], [372,164]
% Set distance threshold to distinguish end points from branch points and
% the range of branch point (within this range there is only 1 branch point)
dist_threshold = {[35 35],[35 35], [15 15],[30 15],[15 10],[15,10], [35,35]};

% Def. channel width
channel_width = {50, 30,12.5,15,20,20, 20};


% Def. variable "maps" with properties
maps = cell(7,1);

for ii = 1:7
    maps{ii} = struct('funct',funct,'Process_Display',proc_disp,'Algorithm',alg,...
        'Animation',animation_switch, 'cost_visual',cost_visual,'save_frame',save_frame, ...
        'logcost', logcost, 'name', mapname(ii), 'goal_loc', goalloc(ii), ...
        'distance_threshold', dist_threshold(ii), 'target_range', target_range(ii), 'scale',...
        scale(ii),'channel_width',channel_width(ii),'NumRob',NumRob,'bolus_region',bolus_region);
end

addpath(genpath(pwd));

% User input: choose a map
fprintf('Choose a map number from the list ...\n')
for ii = 1:size(mapname,2)
   fprintf('Map #%d: %s\n', ii, mapname{ii});
end

temp = input('Input the map name number: ');
testmap = strcat(maps{temp,1}.name,'.fig');

while ~exist(testmap)
    fprintf('File name does not exist!\n')
    fprintf('Choose a map number from the list ...\n')
    for ii = 1:size(mapname,2)
       fprintf('Map #%d: %s\n', ii, mapname{ii});
    end
    temp = input('Input the map name number: ');
    testmap = strcat(maps{temp,1}.name,'.fig');
end

fprintf('Map %s is chosen.\n', testmap);

% Call TestObj to process map and initialize swarm aggregation
obj = TestObj(maps{temp});


