close all
addpath(genpath(pwd));
load('Expt_T01.mat')
localBW = obj.BW;
RGB = double(cat(3, ~localBW, ~localBW, ~localBW));
RGB(:,:,1) = RGB(:,:,1).*182./255+ double(localBW);
RGB(:,:,2) = RGB(:,:,2).*228./255+ double(localBW);
RGB(:,:,3) = RGB(:,:,3).*255./255+ double(localBW);
figure
imshow(RGB);
hold on
layer = 1;
xvar_cnt = 1;
while  xvar_cnt<=size(Expt(:,:,layer),2) && Expt(1,xvar_cnt,layer)>0
    xvar_cnt = xvar_cnt +1;
end
xvar_cnt = xvar_cnt -1;
xvar = Expt(1,1:xvar_cnt,layer);
yvar = Expt(2,1:xvar_cnt,layer);
zvar = mean(Expt(3:12,1:xvar_cnt,layer));
scatter(yvar,xvar,1800,zvar,'s','filled');
colormap(jet(256))
colorbar
caxis([0.85*min(zvar),1.15*max(zvar)])