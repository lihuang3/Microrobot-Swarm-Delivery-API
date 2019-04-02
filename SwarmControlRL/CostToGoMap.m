

load('Map1.txt')
Map = Map1;

ValueMap = Map;                              % 'ValueMap' is a cost-to-go value map

NodeCnt = 1;                                        % Waypoint counter (i.e. node counter) 

% goal = [68,14];
% goal = [11,10];
goal =[5, 8];

ValueMap(goal(1),goal(2))=100;                         % Def the minimum cost to goal
                                                    % Note this is not zero because in "ValueMap", 
                                                    % because initially "0" means it's off medial axes                                                   

frtr = goal;                                        % Breadth-first search (BFS) frontier
 
%% Breadth-first first (BFS) search of all accessible (medial-axis) paths to goal
while ~isempty(frtr)    
    % Extract a 2x2 neighbor (in 'ValueMap') of the current frontier waypoint
    for ii = 1:4
        if ValueMap(round(cos(ii*pi/2))+frtr(1,1),round(sin(ii*pi/2))+frtr(1,2))==1
            ValueMap(round(cos(ii*pi/2))+frtr(1,1),round(sin(ii*pi/2))+frtr(1,2))=ValueMap(frtr(1,1),frtr(1,2))+1;
            frtr = [frtr; round(cos(ii*pi/2))+frtr(1,1),round(sin(ii*pi/2))+frtr(1,2)];
            
        end
    end

    % Remove the current waypoint is from the frontier set
    frtr(1,:) = []; 
end

tmp = ValueMap>0;
ValueMap(tmp) = ValueMap(tmp)-99;
