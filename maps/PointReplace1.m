changeto = [1060 585];
x_obs(idx) = changeto(1);
y_obs(idx) = changeto(2);


mapshow(x_obs,y_obs,'DisplayType','polygon',...
'FaceColor',[182,228,255]./255,'LineStyle','none')
set(gca,'Ydir','reverse')