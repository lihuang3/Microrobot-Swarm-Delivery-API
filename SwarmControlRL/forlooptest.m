run('CostToGoMap.m')

FreeSpace = (find(Map>0));
FreeSpaceIdx = (zeros(max(FreeSpace),1));

for ii = 1:numel(FreeSpace)
   FreeSpaceIdx(FreeSpace(ii)) = ii; 
end
R_FreeSpace = zeros(numel(FreeSpace),1);

for ii = 1:numel(FreeSpace)
    tmp1 = floor(FreeSpace(ii,1)./size(Map,1))+1;
    tmp2 = FreeSpace(ii,1)-floor(FreeSpace(ii,1)./size(Map,1))*size(Map,1);
    R_FreeSpace(FreeSpaceIdx(FreeSpace(ii))) = -(ValueMap(tmp2,tmp1)-1)^2 ;
end


index = 1;

states = uint8(zeros(1,NumRob));%zeros(numel(FreeSpace)^NumRob,NumRob);
states_cap = 1;

iIdx = ones(1,NumRob);


temp = zeros(1,NumRob);

stage = 1;

loop_idx = 1;

loop_mat = ones(2,NumRob);

[states,index,states_cap] = forloopfunction(loop_mat,states,numel(FreeSpace),NumRob,FreeSpaceIdx,FreeSpace,loop_idx,stage,index,states_cap);

states = states(1:index-1,:);  


% % % for i1 = 1:numel(FreeSpace)   
% % %     for i2 = i1:numel(FreeSpace)
% % %         for i3 = i2:numel(FreeSpace)
% % %             for i4 = i3:numel(FreeSpace)
% % %                 for i5 = i4:numel(FreeSpace)
% % %                      temp = [FreeSpaceIdx(FreeSpace(i1)),FreeSpaceIdx(FreeSpace(i2)),...
% % %                          FreeSpaceIdx(FreeSpace(i3)),FreeSpaceIdx(FreeSpace(i4)),FreeSpaceIdx(FreeSpace(i5))];
% % %                      states(index,:) = temp;
% % %                      index = index +1;
% % %                      if index> states_cap
% % %                          states = [states;states.*0];
% % %                          states_cap = size(states,1);
% % %                      end
% % %                 end
% % %             end
% % %         end
% % %     end
% % %  end
% % % 
% % % 
% % % states( states(:,1) == 0,:)=[];  

states_len = length(states);


binSize = 0.25e4;
states_bin_Idx = zeros(floor(states_len/binSize)+1,NumRob);
states_Idx_len = size(states_bin_Idx,1);
jj = 1;
for ii = 1:states_len
    if mod(ii,binSize)==1
        states_bin_Idx(jj,:) = states(ii,1:NumRob);
        jj= jj +1;
    end 
end





% tmp = randi(states_len,100000,1);
% for ii = 1:10000
% 
% x = double(states(tmp(ii),:));
% 
% [~, sIdx] = min(sum( (repmat(x(1:3).*[1e4,1e2,1e0],...
%     [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
% 
% 
% UpBd = min(binSize*(sIdx+1),states_len);
% LwBd = max(binSize*(sIdx-2)+1,1);
% [x0, sIdx] = min(sum( (repmat(x,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
% 
% if sum(( double(states(sIdx+LwBd-1,:))-x).^2,2)~=0
%     disp('Wrong')
%     states(sIdx+LwBd-1,:)
%     x
%     break;
% end
%     
% end