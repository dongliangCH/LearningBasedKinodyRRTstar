function [y1] = NNval40new(x1)
%NNVAL40NEW neural network simulation function.
%
% Auto-generated by MATLAB, 29-Oct-2019 18:27:27.
% 
% [y1] = NNval40new(x1) takes these arguments:
%   x = 6xQ matrix, input #1
% and returns:
%   y = 1xQ matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-3;-3;-1;-1;-1;-1];
x1_step1.gain = [0.333333333333333;0.333333333333333;1;1;1;1];
x1_step1.ymin = -1;

% Layer 1
b1 = [2.6991623794295507288;7.0893515185707087767;-3.2244884772485784552;-1.023400146911372266;10.725802610501149914;-0.014269340344614850694;10.501399146408331831;0.093114411851968670319;0.28532996555798906257;0.17635000394865113504;-1.0226801760280155307;1.0470403265777596946;-0.51883243195781780788;3.3773485141578474256;-0.074937617530187780601;-2.5260134388245356618;-5.5618882596233429894;0.014360767892563092857;0.36725374193200133188;-2.5296517805163292181;0.10175358913721269105;0.40337509074218746985;10.605972778009036261;-3.1057815042369134595;-0.79678208093776570209;1.4771571044563467368;6.9896920265694397756;-0.33303951986643215433;2.5969975589998153254;-0.91754642800849495199;5.5835679614403481708;0.3763258468389109046;0.41567380343764787076;0.25740136561601656418;-0.81567677381012138049;0.31930919780274302777;-1.1081756939104601134;0.98559506358902759793;-11.144506348791603401;0.99518804540460759078];
IW1_1 = [-5.3307580287114380013 -4.6066800736580644227 0.0054976688009943826263 0.26333481251862284056 0.034483789485064579583 2.107049166728191647;1.241553168534031526 -1.3441008228504125466 -1.546617727022004507 -0.85802368886347524235 0.80405012714646850736 1.705728635685974659;-3.1391446244353606865 1.1074118432549897584 2.0817999272023199886 2.7489982217338577719 -0.020322294870534624717 2.5939497836099594252;-0.52175374879447000698 0.18381143852750156387 0.17990972186474901706 0.035596514929934951199 -0.020738909850222350673 0.023815020654095079428;-34.337391603793264494 6.2141709111645306862 -0.68595814697013957062 2.5212433464154675455 -3.2797165097100715592 11.370075661662978561;-4.3053507136936186583 -0.36339702581958049921 -0.44376057282421899064 -0.045076797736809323669 -0.44076729843371165174 -0.0099647465591678630742;-33.404513338284395729 6.0331858511468006157 -0.47998306562004827702 2.6380610832701854029 -3.0681261082541202434 11.165976905924210882;-4.7633750604938756013 -0.40412233769445726761 -0.68862832118413686011 -0.057312828429615575443 -0.66051521191528694832 -0.011204334419799859626;-36.917661868943397963 0.67583434506828621302 -1.6406945470681963961 -0.82796072888354388386 -0.61967145231576159503 0.76777648465260706079;-3.9839551056332114243 0.15223927452532023952 -0.30057874952263308188 0.00395053090242511451 -0.33361089562335449754 0.015463787036962679117;0.54901436162390471196 -0.33585563402524232224 0.29968200231297209557 -0.047480175494968593353 0.19104020891157483253 -0.023288370519512987394;41.081842323696811548 -0.68075719335356299489 0.23679081875254145984 -0.57540871385288072659 1.2329263432902353159 0.097670678416329828608;-4.6801590657646725546 -0.21067853933932026433 0.0053912676161491993043 -0.005246819974566966846 -0.16636532498159159466 -0.01469443102326443637;3.2732042053800265435 -1.1918371794706466016 -2.2339636449927984252 -2.9015985769343179967 -0.015698445651395338818 -2.7592933609175180898;-0.26977988621694398708 0.05180567409504124865 -0.06239679945438077191 -0.020891194617369268638 -0.024505615379599283987 0.019821850029584194702;4.6820872293614650417 3.9154610501516864396 -0.12950079926697188881 -0.42159417465658111102 -0.12068723311928726549 -2.0569506370585335375;-1.0572940589551902413 1.1521000074963425597 0.59301722417879954019 0.80343831796136544465 -0.68894238395871076897 -1.3871233833655527423;3.8731016209657571636 0.10896268607378063742 0.24855477378446408365 0.017177053295259005899 0.29048583497520996666 -0.0025248376384336278407;-40.476711496283421354 0.8041079013284683219 -1.2536928921593417652 -0.85337502056708214315 -0.072138051760462923445 0.73252629403009372755;4.7830905125267477729 4.0417627658131198842 -0.098206789894787885342 -0.37147092053177366289 -0.098117997599031933631 -2.0367750747630575248;-5.0504685001080336093 -0.40425108305310880619 -0.83057037866539329407 -0.064862850873318680933 -0.79125250191830198698 -0.0069766177389493783112;-1.3038414413112233703 0.63921900452705382456 -0.23780942628245560488 0.066780584579113747012 -0.20349267184451175217 0.07479400031782368341;-33.571478267900268122 6.0543368878167553859 -0.33820226696509608777 2.7951540883639061086 -2.9877286351728460367 11.301033820038888678;-3.038226687027352213 1.0374810495001169919 1.9563158990508247026 2.6246186822542076911 -0.052900288515960409985 2.4597211223889225984;-0.2558093739663058197 -0.38850791315410665439 -0.1658124631899034751 -0.097375924382523681988 -0.24665030493333159334 -0.033448354031495521055;0.91280177490781932992 -0.3733704826465253368 -0.066080616580312237507 -0.057983956253512776435 0.066189050725889417293 -0.03590901437526692469;1.3320290326256685187 -1.4558485314737898975 -1.5381567250299594818 -0.92392054027917103198 0.85433461695588119866 1.8115420415642913277;4.2462446209888442539 -0.37167902445107686704 0.37463915030549405083 -0.022586611311695658361 0.39327404375710794948 -0.028465272212219418918;-5.0417525006151979738 -4.3191510263172272488 0.047898107947839087262 0.30629826243400065655 0.063267699405650718036 2.0542984666538144367;0.44262769293233072965 -0.32358173661518924158 0.35543667048229998784 -0.038795271297535696697 0.2086626136765968309 -0.020009709414875051503;1.1996364978855371675 -1.3315158787968912613 -0.69992356944715672373 -0.91011128892849157968 0.77380643498581147544 1.5600510491722665574;2.8117576240775301599 -0.57158964805535239506 0.4162045567041342542 -0.044730900085246758091 0.48376691971771146328 -0.068598812533382458323;1.5524420939643204242 0.61330811122895356746 0.21782115689690281779 0.059994031855114218765 0.20789162596837829078 0.06640256824657024548;-1.2177884688412121683 -0.43456468595414821099 -0.20795325033502692902 -0.05219873672185467578 -0.21426901229874861787 -0.044013873539737152774;0.032010157107906805907 0.68079236131357612205 -0.0088479359562272531098 0.1150294447657917718 -0.02947825430700654406 0.14797366073116482998;-38.204104109879367002 0.73708253165459314271 -1.4428220294588793848 -0.83542937414513385175 -0.34855191468669094457 0.73328231018973499822;-44.396300559345853287 0.74210901150595254272 0.0068423297379216832526 0.66691666048795217314 -1.029118752924676583 -0.075932570626526890711;-15.404464057458799431 0.80966321522275019884 -1.6626319089247618699 -0.0086461996662197618951 -1.6441040497036252255 0.1269067340519939302;35.830628022408070876 -6.4930166809920599746 0.8509374547893081564 -2.4975421491576530464 3.5102405956844768831 -11.787292711890039243;38.510494228820284945 -0.61108629847998208184 0.42305057370995829968 -0.51122876014324247773 1.4242853600386677915 0.12030587027360670205];

% Layer 2
b2 = 2.7504374254632666386;
LW2_1 = [1.9709660532905384223 6.1762239578981299459 1.5734783779374172941 1.5266736958295343829 -21.120585693219727119 6.3119780880786811039 20.632017832698096527 -7.0650137884028101354 -7.9657762946866528253 7.4654681531723836585 3.3127738737674663483 13.29840033583875325 0.44162478146444894556 0.75190178547553820909 2.9921678282666346327 1.9454935595366358303 6.2787356473825184366 7.6445205463757153197 -8.4779945454971521457 -4.7231505994466695952 3.4350415477176072798 -0.6032229206450107073 -8.2566405968309535268 -0.82226426849396561103 0.40863960169364815789 1.4998638165933453337 -3.3984330383994851843 3.0529906166104043841 -4.7503398485975782961 -2.262307866295631964 2.5175994583054839282 -0.21068539843785391574 -0.50600234819290801358 -0.59562139804098124252 0.75914966441346176662 16.463508849420346536 6.5529189849859719175 -0.13164257268355569752 -8.7455664683714200436 -6.7738083434270555117];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.106967270368547;
y1_step1.xoffset = 5.079642;

% ===== SIMULATION ========

% Dimensions
Q = size(x1,2); % samples

% Input 1
xp1 = mapminmax_apply(x1,x1_step1);

% Layer 1
a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*xp1);

% Layer 2
a2 = repmat(b2,1,Q) + LW2_1*a1;

% Output 1
y1 = mapminmax_reverse(a2,y1_step1);
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
  y = bsxfun(@minus,x,settings.xoffset);
  y = bsxfun(@times,y,settings.gain);
  y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
  a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
  x = bsxfun(@minus,y,settings.ymin);
  x = bsxfun(@rdivide,x,settings.gain);
  x = bsxfun(@plus,x,settings.xoffset);
end
