function [y1] = Val1010(x1)
%VAL1010 neural network simulation function.
%
% Auto-generated by MATLAB, 24-Feb-2020 01:28:29.
% 
% [y1] = Val1010(x1) takes these arguments:
%   x = 9xQ matrix, input #1
% and returns:
%   y = 1xQ matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-2.999466;-2.999316;-2.997951;-0.9986092;-0.9998595;-0.9999033;-0.9998944;-0.999858;-0.9996955];
x1_step1.gain = [0.333432195979441;0.333424914043057;0.333616852054771;1.00178849299655;1.00040101074516;1.00027532578342;1.00091358387368;1.00062243718705;1.0007487602224];
x1_step1.ymin = -1;

% Layer 1
b1 = [-0.64148235529514407993;0.45432762322895625662;0.80072815827192134197;0.77752385338203866549;0.70551491001390997937;-0.82163435139423945941;0.71803625125807857277;-0.74538968446318443206;0.65336072303419157059;0.78098524084105525045];
IW1_1 = [-0.065799609279609372336 -0.45219667243540462298 -0.051310850698949514448 -0.037494563048818048412 -0.39817956387072322988 -0.032037773315945372588 -0.049637836133778139025 -0.37651379900112769139 -0.0071121977306230427587;-0.28957566858315036962 -0.17046129869315662653 0.21807911942359092805 -0.23469366542124808772 -0.20196885829812716295 0.14442388918261625519 -0.19657271592962105577 -0.13068488168487962797 0.15972187217405126036;-0.95329007587856862127 -0.4723087271990790148 0.5461466399930355875 -0.39129690397265726176 -0.16283552427468425683 0.27739141769342268162 -0.29810160342839392422 -0.127963806462255969 0.24637265273303154967;-0.081022090198246507486 -0.06796164797953908876 -0.46601629237895586533 -0.079975581590834191048 -0.10533929569550078142 -0.10737869738865749025 -0.045093680310996982208 0.0046432379341471846645 -0.50570788962253054866;-0.069602374295243366054 -0.027643292857234946086 -0.44990444034546811514 -0.08064474817751604574 -0.059575299794568312151 -0.53761400170178330615 -0.0027055199373373730633 0.027358643333236270373 -0.042319756952750614554;-0.35768593665168901818 0.097494356928604516965 -0.092127764546485274577 -0.25016145861580169063 0.097570848129753792732 -0.060163680007751782486 -0.31428166274391139412 0.062576635801454491559 -0.053757483877974326925;0.10890055904851156376 1.0538462742119563131 0.13258778437643997661 0.027063083053683798657 0.45791689159415865928 0.15506189646471613131 0.11205577134112837401 0.45469886540936998998 0.10284382117966692216;-0.9766589793516501361 0.53689972008429265315 -0.37258326086898541085 -0.33191385444072546429 0.19572043686681575569 -0.2417562556937088436 -0.47488474957931620102 0.11410512955150192704 -0.22822130413725225107;-0.084732403831636798897 -0.093511174139513264514 -0.054409449582496334508 -0.055719903430826743518 0.36552651255479878811 -0.024166691537172768633 -0.06331672531289110617 -0.48890036141935588887 -0.026496473423581082957;0.044869121520642346601 -0.056409903525340934771 0.039299940398447627543 0.48654625738870060925 -0.077105569954381822373 0.017531666617347053255 -0.44950174895004951781 0.033807056729107075321 0.071531357848628934759];

% Layer 2
b2 = [-2.6144982068991851065;4.7597158784052018277;0.26177372340569082798;-0.92572448777404914289;1.0475252240677537241;1.1089981037965543909;-0.91958437956760297194;0.036220414648486243259;-1.4745908365889455194;0.74953594932297851194];
LW2_1 = [-0.10621335487508640205 0.050460567503235138898 0.34769598042293475837 0.80521535831122859062 0.56443454127971981471 -0.48299815095819609301 0.59676953937108745407 -0.5310997992329944406 0.34019360225965045164 0.24843562363966312234;-3.5325141473145182935 2.6694567486041700377 -2.6203518794966820238 -1.9313061380702019054 -1.7391536898558959123 -2.9588524199653347679 -3.5474790327945244783 2.7114289583916719728 -0.64104121185955909024 -0.8673044189628966727;-1.0837541813869671081 0.64189211501083742739 0.24625612756205039755 -0.4609668895785615228 -0.2627428672098101603 0.45036898345224435358 0.16136269380919848748 -0.01235175001654714548 -0.061783383232267626017 -0.033317383220543762834;0.54085673162732483821 -0.14797891630337617341 -0.31498892818684010031 0.30866067693558452145 -0.099840487213495199281 -0.68628000405547573415 -0.066513268378310161366 0.48154284265814900623 0.12692452911983315156 0.14000903971089256883;-0.17721672591632350868 -0.87542999449165237458 0.17843256312202626646 -0.13211322576328407563 0.033274270901105552911 1.185574856143803979 -0.074842706872591605038 -0.88339074785508975296 -0.068248049394564067049 -0.067543876419686371682;-1.7834050458067816081 -0.11487504736239595449 0.37508160360576142667 -0.71833514881611981728 -0.3934920953150923828 1.0193104200132616644 -0.10512267605936587156 -0.11290976997428588124 -0.13168566178236826292 -0.15869344440240906735;-0.068146094753774782982 0.70226901230869420711 0.38524995402076028794 -0.3155849395841808791 0.052482749509209551675 -0.79884797692133657776 0.00081637455924476087961 -0.38613987460636611804 0.2517479157377305321 0.21496318651558879442;0.39952942359945803341 -0.64667148556687514827 -0.52330095321191827384 0.70752768998747339602 -0.19318917485290507119 -0.17259531392358035129 -0.30097947096581062487 0.51593564594860064076 -0.09110021505245979867 -0.086372624997515565215;1.7465871915505439826 -1.635859941161202924 1.0216106841202676758 0.65271065264171712617 0.59743791985976812953 1.4001466178790271666 1.4492836222085054843 -1.171098561168370189 0.28680068035001221594 0.32330923503653219431;0.32565900735502856245 0.12485820771203007262 0.1569209895342103811 -0.31094496511979213826 -0.35787709474027001866 0.28242513208431091476 0.38954506635743957332 0.23096005483554929438 -0.10612541109776567338 -0.14061582479934411904];

% Layer 3
b3 = -1.4032852916864870796;
LW3_2 = [-2.3486226574453441529 3.2734520414324506099 -1.7324956939359783537 1.538508969290264039 1.0859449045392430833 0.76753114187698989745 -0.5735962700066488118 -0.77891987891718450499 2.470132977963181542 1.5344701865850107758];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.208146846767807;
y1_step1.xoffset = 2.226429;

% ===== SIMULATION ========

% Dimensions
Q = size(x1,2); % samples

% Input 1
xp1 = mapminmax_apply(x1,x1_step1);

% Layer 1
a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*xp1);

% Layer 2
a2 = tansig_apply(repmat(b2,1,Q) + LW2_1*a1);

% Layer 3
a3 = repmat(b3,1,Q) + LW3_2*a2;

% Output 1
y1 = mapminmax_reverse(a3,y1_step1);
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
