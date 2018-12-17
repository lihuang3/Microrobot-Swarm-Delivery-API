% mazeData = int16(obj.BW);
filename = 'map1203.txt';
mazeData = load(filename);
costMap = mazeData;
goal = [137, 47];% [393, 137];
BSF_Frontier = goal;
cost = 100;
costMap(goal(1),goal(2)) = cost;

num = size(BSF_Frontier);
num = num(1);
while num>0
    cost = costMap(BSF_Frontier(1,1),BSF_Frontier(1,2))+1;
    for i=1:4
        new_pt = BSF_Frontier(1,:) + [cos(i*pi/2), sin(i*pi/2)];
        if(costMap(new_pt(1),new_pt(2)) == 1)
            BSF_Frontier = [BSF_Frontier; new_pt];
            costMap(new_pt(1), new_pt(2)) = cost;
        end
    end
    BSF_Frontier(1,:) = [];
    num = size(BSF_Frontier);
    num = num(1);
end

costMap(costMap>0) = costMap(costMap>0)-99;
max_dist = max(costMap(:))