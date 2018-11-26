figure(2); clf
x_obs = [311; 355; 355; 311;NaN;311;311;170;81;32;31;52;52;93;135;135;154;154;107;177;241;204;204;222;223;256;294;292;315;314;266;202;334;451;385;339;335;355;355;396;429;429;449;449;411;475;545;499;498;517;518;560;599;599;620;620;569;478;355;355;645;645;0;0;NaN];

y_obs = [0;0;60;60;NaN;0;132;186;255;380;426;424;380;286;383;424;424;381;269;215;269;384;425;425;383;287;380;426;425;384;256;207;157;207;258;383;425;426;382;285;380;426;426;378;269;216;269;382;425;425;381;286;382;423;424;380;257;189;134;0;0;455;455;0;NaN];

mapshow(x_obs,y_obs,'DisplayType','polygon',...
'FaceColor',[182,228,255]./255,'LineStyle','none')
set(gca,'Ydir','reverse')

axis([-0.01 645.01 -0.01 455.01]);
axis off

hold on
mapshow([0 645 645 0 0],[0 0 455 455 0],'LineWidth',2,'Color','black')

