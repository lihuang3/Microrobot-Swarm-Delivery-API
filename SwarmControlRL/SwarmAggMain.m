mode = 1; % 0 for restart, 1 for continue

close all
clc
rng(35)
cnt = 0;

pause_timer1 = 0.4;
pause_timer2 = 0.3;
process_display = 1;

learnRate = 0.65;

epsilon = 0.0;

epsilonDecay = 1;%0.99;

discount = 0.65;

successRate = 1;

winBonus = 100;

maxIter = 10000;

maxStep = 100;


if mode == 0
    
    load('mapT.txt')
    Map = mapT;
    bw = Map>0;
    
    FreeSpace = find(Map>0);
    R_FreeSpace = zeros(max(FreeSpace),1);

    for ii = 1:numel(FreeSpace)
        tmp1 = floor(FreeSpace(ii,1)./size(Map,1))+1;
        tmp2 = FreeSpace(ii,1)-floor(FreeSpace(ii,1)./size(Map,1))*size(Map,1);
        R_FreeSpace(FreeSpace(ii)) = -(Map(tmp2,tmp1)-1)^2 ;
    end

    index = 1;
    NumRob = 5;
    states = zeros(1,NumRob);%zeros(numel(FreeSpace)^NumRob,NumRob);
    states_cap = 1;


    for i1 = 1:numel(FreeSpace)   
        for i2 = i1:numel(FreeSpace)
            for i3 = i2:numel(FreeSpace)
                for i4 = i3:numel(FreeSpace)
                    for i5 = i4:numel(FreeSpace)
                         temp = [FreeSpace(i1),FreeSpace(i2),FreeSpace(i3),FreeSpace(i4),FreeSpace(i5)];
                         states(index,:) = temp;
                         index = index +1;
                         if index> states_cap
                             states = [states;states.*0];
                             states_cap = size(states,1);
                         end
                    end
                end
            end
        end
     end
     
     states( states(:,1) == 0,:)=[];
   
    
    

    actions = [1 2 3 4];

    R = sum(R_FreeSpace(states),2);

    Q = repmat(R,[1 length(actions)]);

    Q0 = Q;

    V = zeros(length(states),1);
    
    RobLoc = FreeSpace(randi(length(FreeSpace),NumRob,1));

    LocCol = floor(RobLoc./size(Map,1))+1;
    LocRow = mod(RobLoc,size(Map,1));
    LocRow(LocRow==0) = size(Map,1);
    z0 = RobLoc';
end
%???Vorig = reshape(max(Q,[],2),[7,7,7]);





fig = figure; hold on

fig.Position = [250 100 800 600];
axis([-1 size(Map,2)+1,-1 size(Map,1)+1])
[row, col] = find(Map>0);
hSpace = scatter(col,row,2000,'filled');
hSpace.CData = [1 1 0];

hRob = scatter(LocCol,LocRow,200,'filled');
Loc = [LocCol,LocRow];

fp1 = plot(0,0,'b','LineWidth',5);
fp2 = plot(0,0,'*b','MarkerSize',25);
plot(0.001,0,'.k','MarkerSize',30,'MarkerFaceColor',[1 0 0]);




 
for iIter = 1:maxIter
    %%
    RobLoc = FreeSpace(randi(length(FreeSpace),NumRob,1));
    RobLoc = sort(RobLoc);
    LocCol = floor(RobLoc./size(Map,1))+1;
    LocRow = mod(RobLoc,size(Map,1));
    LocRow(LocRow==0) = size(Map,1);

    z0 = RobLoc';

    Loc = [LocCol,LocRow];
    

    if process_display == 1
        hRob.XData = Loc(:,1);
        hRob.YData = Loc(:,2);
        pause(pause_timer1);
    end
    
    z1 = z0;
   for iStep = 1:maxStep
     
      [~,sIdx] = min(sum((states-repmat(z1,[length(states),1])).^2,2));
      
      
      
       if (rand()>epsilon || iStep == maxIter) && rand()<=successRate % Pick according to the Q-matrix it's the last episode or we succeed with the rand()>epsilon check. Fail the check if our action doesn't succeed (i.e. simulating noise)
           [~,aIdx] = max(Q(sIdx,:)); % Pick the action the Q matrix thinks is best!
       else
           aIdx = randi(length(actions),1); % Random action!
       end
      
       T = actions(aIdx);
       
       Last_Loc = Loc;
       
       Loc = Loc + repmat(( round([sin(T*pi/2) cos(T*pi/2)])),[length(RobLoc),1]);
        

       
       % Collision Check
      
       collision_idx =find(diag(bw(Loc(:,2),Loc(:,1)))==0);
      
       Loc(collision_idx,:) = Last_Loc(collision_idx,:);
       
       if process_display ==1
           set(fp1,'XData',[0 sin(T*pi/2)]);
           set(fp1,'YData',[0 cos(T*pi/2)]);
           set(fp2,'XData',sin(T*pi/2));
           set(fp2,'YData',cos(T*pi/2));

           hRob.XData = Loc(:,1);
           hRob.YData = Loc(:,2);
           pause(pause_timer2)
       end

       % Update State
       z1 = (Loc(:,1)-1)'*size(Map,1)+Loc(:,2)';
       
       [z1, z1_idx] = sort(z1);
       Loc = Loc(z1_idx',:);
       
       [~,snewIdx] = min(sum((states - repmat(z1,[length(states),1])).^2,2));
       
       
       if R(snewIdx) == 0
           success = true;
           bonus = winBonus;
       
       else
           
           bonus = -1;
           success = false;
       end
       

       
       if iIter < maxIter
          
           Q(sIdx,aIdx) = Q(sIdx,aIdx) + learnRate * ( R(snewIdx) ...
               + discount*max(Q(snewIdx,:)) - Q(sIdx,aIdx) + bonus );
           
           
       end
       
       
        if success
            break;
        end
         
  
   end
   epsilon = epsilon*epsilonDecay;
   
   if iStep < 30 && iIter > 0.95*maxIter
        disp('Finishing Step = ')
        disp(iStep)
        disp(iIter)
        cnt = cnt  +1;
   end

   
end
   disp('Success times')
   disp(cnt./maxIter*100)
   
   diff_Q = Q-Q0;