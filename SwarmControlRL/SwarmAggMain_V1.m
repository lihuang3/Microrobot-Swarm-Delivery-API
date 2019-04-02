mode = 0; % 0 for restart, 1 for continue

%%%
%  load('20170909_6R_StdMap.mat')

close all
clc
time_rec = datetime
cnt = 0;

pause_timer1 = 0.4;
pause_timer2 = 0.4;
process_display =0;


learnRate = 0.6;


epsilon = 0.5;

epsilonDecay = 1;%0.99;

discount = 0.8;

successRate = 1;

winBonus = 100;

maxIter =50000;

maxStep = 300;


if mode == 0
    
    NumRob = 3;
    run('forlooptest.m')
    

    actions = [1 2 3 4];

    R = sum(R_FreeSpace(states),2);

    Q = repmat(R,[1 length(actions)]);

end
%???Vorig = reshape(max(Q,[],2),[7,7,7]);

%      RobLoc = FreeSpace(randi(length(FreeSpace),NumRob,1));
     RobLoc = [20; 14; 17; 113; 110; 116];
    
    LocCol = floor(RobLoc./size(Map,1))+1;
    LocRow = mod(RobLoc,size(Map,1));
    LocRow(LocRow==0) = size(Map,1);
    z0 = RobLoc';



fig = figure; hold on

fig.Position = [250 100 800 600];
axis([-1 size(Map,2)+1,-1 size(Map,1)+1])
[row, col] = find(Map>0);
hSpace = scatter(col,row,1000,'filled');
hSpace.CData = [1 1 0];

hRob = scatter(LocCol,LocRow,200,'filled');
Loc = [LocCol,LocRow];

fp1 = plot(0,0,'b','LineWidth',5);
fp2 = plot(0,0,'*b','MarkerSize',25);
plot(0.001,0,'.k','MarkerSize',30,'MarkerFaceColor',[1 0 0]);




 
for iIter = 1:maxIter
    %%
    

%     RobLoc = FreeSpace(randi(length(FreeSpace),NumRob,1));
     RobLoc = [20; 14; 17; 113; 110; 116];
    % Trained
    % [68; 14; 53; 87; 110; 58];
    % [59; 14; 17; 110; 90; 116];
    % [20; 14; 17; 113; 110; 116]; 
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
    z1 = FreeSpaceIdx(z1)';
    
           [~, bnewIdx] = min(sum( (repmat(z1(1:3).*[1e4,1e2,1e0],...
                   [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
        UpBd = min(binSize*(bnewIdx+1),states_len);
        LwBd = max(binSize*(bnewIdx-2)+1,1);
       [~, snewIdx0] = min(sum( (repmat(z1,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
        snewIdx = snewIdx0 + LwBd-1;
    
    if process_display ==1
        figure(2)
        fp3 = plot(0,R(snewIdx),'-k','LineWidth',2);
    end
   
    for iStep = 1:maxStep
     
       [~, bIdx] = min(sum( (repmat(z1(1:3).*[1e4,1e2,1e0],...
                   [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
        UpBd = min(binSize*(bIdx+1),states_len);
        LwBd = max(binSize*(bIdx-2)+1,1);
       [~, sIdx0] = min(sum( (repmat(z1,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
        sIdx = sIdx0 + LwBd-1;

       if (rand()>epsilon || iStep == maxIter) && rand()<=successRate % Pick according to the Q-matrix it's the last episode or we succeed with the rand()>epsilon check. Fail the check if our action doesn't succeed (i.e. simulating noise)
           [~,aIdx] = max(Q(sIdx,:)); % Pick the action the Q matrix thinks is best!
       else
           aIdx = randi(length(actions),1); % Random action!
       end
      
       T = actions(aIdx);
       
       Last_Loc = Loc;
       
       Loc = Loc + repmat(( round([sin(T*pi/2) cos(T*pi/2)])),[length(RobLoc),1]);
        

       
       % Collision Check
      
       collision_idx =find(diag(Map(Loc(:,2),Loc(:,1)))==0);
      
       Loc(collision_idx,:) = Last_Loc(collision_idx,:);
       
       if process_display ==1
           set(fp1,'XData',[0 sin(T*pi/2)]);
           set(fp1,'YData',[0 cos(T*pi/2)]);
           set(fp2,'XData',sin(T*pi/2));
           set(fp2,'YData',cos(T*pi/2));
           set(fp3,'YData',[fp3.YData,R(snewIdx)]);
           set(fp3,'XData',[fp3.XData,iStep]);
           hRob.XData = Loc(:,1);
           hRob.YData = Loc(:,2);
           pause(pause_timer2)
       end

       % Update State
       z1 = (Loc(:,1)-1)'*size(Map,1)+Loc(:,2)';
       
       [z1, z1_idx] = sort(z1);
       Loc = Loc(z1_idx',:);
  
       z1 = FreeSpaceIdx(z1)';
       [~, bnewIdx] = min(sum( (repmat(z1(1:3).*[1e4,1e2,1e0],...
                   [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
        UpBd = min(binSize*(bnewIdx+1),states_len);
        LwBd = max(binSize*(bnewIdx-2)+1,1);
       [~, snewIdx0] = min(sum( (repmat(z1,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
        snewIdx = snewIdx0 + LwBd-1;

       
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
   
%    if iStep < maxStep %&& iIter > 0.95*maxIter
% 
%         disp('Finishing Step = ')
%         disp(iStep)
%         disp(iIter)
%         cnt = cnt  +1;
%    end
   
   if mod(iIter,500)==1
       disp('Iteration =')
       disp(iIter)
   end
   
end
   time_rec(2) = datetime
