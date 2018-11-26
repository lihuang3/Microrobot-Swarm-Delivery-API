x_obs = [xtmp; NaN; x_obs];
y_obs = [ytmp; NaN; y_obs];

figure
mapshow(x_obs,y_obs,'DisplayType','polygon',...
    'FaceColor',[182,228,255]./255,'LineStyle','none')
set(gca,'Ydir','reverse')