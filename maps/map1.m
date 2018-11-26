
figure(2);clf
axis([-0.01 1050.01 -0.01 850.01]);

x_obs = [0;1051;1051;0;0;58;57;199;205;213;275;359;415;465;506;603;618;...
    614;453;269;270;428;473;507;486;695;832;998;998;874;868;881;922;...
    929;929;894;893;853;837;825;820;810;767;713;690;674;743;742;676;568;569;...
    533;526;354;357;318;320;225;143;57.29;NaN;290;260;245;237;281;297;...
    288;292;NaN;354;318;312;314;341;365;374;374;365;355;NaN;...
    394;395;404;460;485;501;504;455;414;397;NaN;258;281;321;342;354;...
    386;403;403;394;373;345;260;260;NaN;319;319;355;387;409;413;395;343;319;NaN;...
    392;392;412;433;454;476;485;470;441;414;401;392;NaN;518;562;573;584;604;604;578;...
    550;522;504;503;519;NaN;434;433;447;468;496;535;566;566;509;478;450;435;NaN;...
    649;649;690;780;828;828;824;695;651;NaN;447;447;463;570;618;640;690;705;705;...
    675;571;521;470;446;NaN;532;538;575;596;598;590;577;566;534;NaN;620;615;623;...
    652;675;675;645;620;NaN;607;607;622;662;684;684;671;669;622;616;608;NaN;...
    690;695;718;762;781;798;798;757;718;693;NaN;791;791;826;854;878;883;855;813;792;NaN;...
    730;726;727;736;780;805;824;823;802;734;NaN;695;695;710;766;781;782;765;748;724;702;696;NaN;...
    791;789;810;826;850;871;871;867;839;816;792;NaN;863;888;908;910;896;867];

y_obs = [850;850;0;3;850;528;451;462;445;419;340;287;238;230;242;226;214;207;164;...
    145;71;78;40;58;88;139;134;131;209;211;239;289;377;397;442;491;544;596;621;...
     645 ;652; 663;684;689;706;723;729;803;802;772;793;789;760;657;694;693;651;570;538;532;NaN;...
    361;389;412;471;450;422;389;361;NaN;313;341;371;400;400;379;353;325;313;312;NaN;...
    297;316;325;318;309;288;268;251;270;294;NaN;494;483;483;490;497;508;527;578;...
    591;590;564;501;494;NaN;439;455;481;492;492;444;412;413;437;NaN;365;389;410;416;...
    417;397;369;345;340;345;352;363;NaN;378;379;364;313;275;259;261;271;299;336;353;377;NaN;...
    494;539;552;552;540;492;430;417;413;422;453;492;NaN;253;299;334;315;263;228;222;212;253;NaN;...
    608;628;644;698;713;713;674;647;610;580;597;585;584;607;NaN;553;569;575;552;493;482;483;509;549;NaN;...
    520;547;564;563;543;520;505;515;NaN;382;442;470;497;497;465;439;367;326;355;382;NaN;...
    366;438;473;475;465;430;386;348;345;364;NaN;328;346;392;392;371;333;304;309;324;NaN;...
    614;632;656;665;662;648;614;590;581;608;NaN;551;566;578;577;569;544;514;499;497;507;551;NaN;...
    480;526;552;562;561;515;469;455;430;439;477;NaN;411;443;438;398;395;406];


mapshow(x_obs,y_obs,'DisplayType','polygon',...
    'FaceColor',[182,228,255]./255,'LineStyle','none')

hold on

mapshow([0 1050 1050 0 0],[0 0 850 850 0],'LineWidth',2,'Color','black')
