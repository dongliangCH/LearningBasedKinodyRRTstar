function [y1] = Val2040(x1)
%VAL2040 neural network simulation function.
%
% Auto-generated by MATLAB, 24-Feb-2020 01:05:36.
% 
% [y1] = Val2040(x1) takes these arguments:
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
b1 = [-0.68862167842446264654;-0.99420484345552195826;-0.090631789691209552196;0.47528036275654972354;0.70222265494661573193;-0.017592716730786512463;0.32820899101004918919;0.78546048104242960353;0.69143847148004167202;0.36053468086115819657;-0.57481566941449968589;0.70439616953086903184;-0.81338491130193979473;-0.68229954382077295794;-0.52958326296549518286;-0.054589587062421923713;0.83500173548020684144;-0.65792533723661095202;-0.029864442246184771945;0.46705096304581356081];
IW1_1 = [-0.0079713885170393967983 -0.0146015874209557521 0.00069454717235767002292 0.0081409087260882282561 -0.24005785001779245125 -0.0093875574657132244277 -0.0027330493924065542809 0.26420702902572396065 -0.025935337328739736101;-0.011501058896818150759 0.13945718017801286837 0.013467706172330109679 0.028864949275980267274 0.32818488565298659099 0.23081746264785010792 0.050032363224318421313 0.32108939785741791306 0.20460181066453830834;1.2406955833903354769 -0.23423231509176420384 -3.2919204071269843048 0.33111373593594656262 -0.035829548034681059365 -0.81978997836472311533 0.33450753413070316578 -0.029362563584087866125 -0.83397186150941693406;-0.64633531971395630222 0.47141785191772700436 0.6042043314193168424 -0.23669563964936712885 0.1990961586969979269 0.25496196512645541432 -0.24664862674447399216 0.21174495021118894256 0.2541576934296455037;-0.0016987278457661235051 0.017414338740556967872 -0.011995468841701219589 0.15799656286134322603 -0.07700883970742426754 0.13244252974123446598 -0.18812380060146169369 0.063314537228516820644 -0.11124994674941146577;1.4938135884708378942 -3.4030605104922670989 -0.016760653744876282539 0.18069589878053518572 -0.80924064921023242114 -0.021795236560118065594 0.27605317683814595275 -0.78233887586538530812 0.098057363726001184334;0.2307628291871497983 0.45636005973334020647 -0.74651660743477776805 0.07895749718513767057 0.2082767484115150558 -0.23841721845476379626 0.083741273982737382542 0.19538814578571714331 -0.27354252157876512985;-0.031345274454908533812 -0.26757205406095085909 0.01991162673543058273 0.029079596365008324871 0.15678578374440010901 0.07734686493691064757 0.034022485500351890009 0.1421169393845448381 0.13247317291791091409;-0.075072772038285162655 0.04033326674122311023 0.053350851712580846242 0.2980509326742518672 -0.061798268785126682368 0.18444790246978942649 0.30545507087495599174 -0.060485260479372859876 0.15786648946399942095;-0.050313709363578082434 -0.92651834180332748314 -0.026237974407205238991 -0.019028554694662892788 -0.34263880420353143297 0.013519999274337226106 -0.017806632271474567841 -0.34437862248333389603 -0.00022308662713384832263;0.16446124696492550599 -0.15665774997467871166 -0.025232670122137826341 0.16611746427340551291 -0.16571844530828594433 0.013497616225911919854 0.17507904418032571581 -0.15882234145925439028 0.024497951533195217594;-0.021293580040955974175 0.010734524397375291155 0.024772885851382934336 -0.23961510529801027802 -0.088343851170952239493 0.11426271550479200068 0.16571373446593773426 0.063280295576920803091 -0.022641944758969294316;-0.10366497499887671052 0.016920084954893645224 0.26246026748715656218 -0.1179608482647602069 -0.021592557294947335722 0.23692003296106495247 -0.14309109266149921957 -0.024158328695106753153 0.26310921027081424217;0.00017439562906398239591 -0.010994088483421640343 0.022005147606558483203 -0.022430129634512026293 0.063366884323597452711 0.21757172716037345706 0.036983496566386594995 -0.070787400221951748969 -0.23000430629088911005;-0.23810570894019902899 -0.038129095923231604015 -0.135493537789831614 -0.21085703879879891254 -0.014129903651245476898 -0.14676945074153505133 -0.21035668068889448734 -0.023199632260661681021 -0.13285854508310085409;-3.4101550114011844883 -2.2256807125459396879 0.12741916260543814654 -0.67746613038167602383 -0.57019881319901866146 -0.10750929101273169752 -0.7530348721186858052 -0.7622408408655250156 -0.10823932607040750997;-0.013719295693946882003 0.065173365431078850163 -0.17112839463334175716 -0.20244993044322243114 -0.16651217221776060118 0.29268183730441182933 -0.1812120068908812387 -0.14049557703559090172 0.28558781762041546282;0.13453404778327168745 0.26636473341356620148 0.031186656470698116 0.12170898038118774243 0.22634924370930287196 -0.048135516542106418503 0.12911478404259502106 0.22019668063266872871 -0.038960687289307911751;-1.4034644168341932069 0.48988290606092937951 -2.1572676509973045533 -0.41512930840051059933 0.098192858911124711319 -0.5364752503320434629 -0.41739602090344712915 0.10351165166436457343 -0.54796555486848319561;-0.59467633917857343206 -0.03861135682179211992 -0.65117062800119207555 -0.25011364459236551383 0.0038099810066596160457 -0.23116399244834429094 -0.2512251448476232607 0.0029573154859622906646 -0.2256784384596508819];

% Layer 2
b2 = [-0.98508124418527265309;0.46453529905990986748;0.65130092604216238694;0.037200012101640485274;0.012245419601817041089;-0.94639137858502964118;0.50265056443580835577;0.48472701031524539905;-0.034241507338966949148;0.27939224546346297373;0.16962689935141342379;0.35224994548811316974;-0.40113588275193967636;-0.26603424395680491665;-0.79494980365180445414;-0.34054287844484354419;0.71912453466125902946;0.7666417549536813647;-0.88477711283714000068;0.044446808016444856737;-0.75395028631718974399;0.49357741580597463971;0.66088086373453447386;-0.32321401891800849437;-0.38718201824333126071;-0.35301732835394916687;0.21205964810543276311;0.44180973738844142584;-0.22839976884195004048;0.042946809163833477019;0.53109341487714478713;-0.87151116557115959615;-0.0088743530356775682938;0.096475554194766846572;-0.42268576094329923798;0.55470945062124854541;-0.46062386951246414579;-1.1125065195791901118;0.12233169540509485074;1.4505186976954069777];
LW2_1 = [0.19118803451842872332 -0.12812811768328036255 0.023883738330604753486 0.42705647246340783241 -0.11242377165205197365 0.027785156330936588986 -0.19904869889498080782 -0.40922931035635284891 -0.023061478332847305112 0.50779503999878239373 -0.45093956648267541354 -0.16944638587920021755 -0.31974667518037858116 0.041239596193363439736 -0.75089568925461502857 0.0075168875915772826943 0.13505452876822365837 -0.20846635964165483035 -0.10324437247352158398 0.41012604946485753388;-0.1134662032544403315 0.33140298571230680791 -0.12901975211453159398 0.19668478737302394488 0.051931952493590977094 -0.1717653292724747871 -0.29051100613343588952 -0.11290086911438486361 0.047969917246962139046 -0.29956088441035300107 0.13809869451150663666 0.11814828191080765929 0.31973448819833993895 0.021019289444798389122 0.40083532582145309231 0.10689629766018744761 -0.12555553715025619121 0.24900972305038293619 0.21966947681466131304 0.15583711656471957441;-0.11625969432528135417 0.13161142094850411532 0.28598771023059205154 0.1761406406577839534 0.17764438665336537704 -0.099496703170871406141 -0.44538048879754871257 0.23610187981891087028 0.067699374565955719452 -0.47405256976398968405 0.060663480623149822202 0.11219879358972842387 0.014642535870596879236 -0.036185037509632166086 0.47680039574615828979 -0.21770081629138463009 0.16801433470680057769 0.0406392425412192837 -0.14475747227078683554 -0.36163419563365772902;-0.03094640708523887021 0.17318331429655672848 -0.065786695945897200044 -0.33292792278345262336 0.2855623812272770623 0.095586407434287853913 -0.17510655231724911451 -0.24872319255850841579 -0.10711747750175307226 0.031302815880408847715 0.23251211442807873109 0.21288434468789363407 0.50409049426912688752 -0.11391521842193874647 0.04345052277066954044 -0.15471880058833484273 -0.16758380179703441515 0.025598209696434934268 0.37659071311015684946 -0.40048923954068571529;0.14999703229948585514 0.052316805250093842417 0.10078693794287646446 0.054653078623349879428 0.063641597225213675237 0.3079803952546038559 -0.058121979810605274575 0.046656776280244796618 0.025539158400042248631 -0.031637965480913375627 -0.22093556628328625546 -0.085402727675616699199 -0.13277153570998104226 0.19639184165451847863 0.071334152444255213843 -0.080242809249407562322 0.21267220866108707544 0.0045275017802164467529 -0.34311177631171085878 -0.072815891867039100882;-0.63949775683323106801 0.31287498567397442617 0.40388121373529090041 0.30349493066296806543 0.78454404085697582172 -0.44984329733294903253 0.14107504411468307914 -0.41983817878731827244 -0.6005678412853253656 0.51923709926112260327 0.41197798276313979393 0.76277261456350109459 -0.0044411768637846547186 -0.6969559963867721919 -0.81716598343459723175 0.42435382497164242377 -0.4229123730519810942 0.18537750175089418025 -0.41281724487048671035 0.1567386845768117265;-0.14327424019102152908 0.5165370257326344916 0.13299586180721506556 0.35182911410548811659 0.25401658738554055983 -0.077827288450227530281 -0.1901397508323865726 -0.26252048345588513412 -0.23262778844573259174 0.32854138244550423753 0.98735454998936800219 0.15268075523327240273 0.28919736570826853894 -0.13187290696971271098 0.17670022360521989158 -0.025405247075116498684 -0.028984210884593117341 -0.070456190704430390892 0.032969823553115111536 0.083754155219230339013;0.43441306125843681674 -0.27783557178269974175 0.42460439855099851858 -0.2428706366936524419 -0.53956414246816741809 0.40863390120046905185 -0.013997732413353109263 0.3033058002193793623 0.42636932012740907938 -0.26868381100865829092 -0.20849105128028033729 -0.5294742733216668551 -0.21998933468716480588 0.4651489549982015248 0.66666602209953274016 -0.37756519934437582364 0.4054194184634840048 0.047748418617701002264 -0.49592436514436233352 -0.040682714586022995784;0.00012731038716326504266 -0.021588691515336138477 -0.18563632356516601707 -0.15831267620791544815 0.093598490346969764886 0.19514567945097169299 0.33062515699697697658 0.092497896419616768005 -0.42935726861394379883 -0.041474286244647821931 0.11529228142134906321 0.071667072123029709751 0.28811347266719000704 0.12312811850729471486 0.061410057831175174359 0.036005091079134986243 0.27003966215434260789 -0.064158413894368770958 0.15774586765917492093 -0.030954121038793493831;0.4602160461633617583 -0.18250112290525016823 0.18244629747245860729 -0.06723431716239229583 -0.39422952370657277088 0.42638766287748847139 0.050480005842260623761 0.13753958261215312731 -0.22836925732618273965 0.28009406006152404167 -0.12841038978063840093 -0.4587889482568880406 0.06832318321984287568 0.35840679056706548078 -0.39497203946295167398 0.26532679187076568805 0.074589677280737426424 -0.024428642975091487133 -0.40485037311497956702 0.0081071311694662969577;-0.34301893910217345907 0.37700012210696220993 0.0054483147007989059935 -0.20989882407192819724 0.31425297384290273861 0.11243177529506676038 -0.054265702765202393498 0.13149360397837450698 -0.090992566888618939269 -0.051311558592393689826 0.034063816843099524456 0.31702593583607741046 0.39603773397080149765 -0.28783589810589077507 0.31976888424812838219 0.096857561054950261359 -0.096168811736385501066 0.51080519840027749368 -0.23858187326191557243 0.25660352673402880841;0.41446578789380666974 -0.22162246380007105673 -0.28779163242140898271 -0.1316693098815021612 -0.48293130878001705453 0.29941099789915515128 -0.28944150555935793667 0.17477711697042117023 0.22396865458333220511 -0.15133315506811848006 -0.29294117618335929754 -0.53812206293005204749 -0.11796288839066371157 0.45588302476723036039 0.12522468067102493494 0.4168063619118793639 0.47261807615939210869 0.050498552692542532372 -0.45609847089711630375 0.0965780126820193624;0.1804643268082204155 0.096372774052594978311 -0.16553605950739716524 -0.07238774447546643398 -0.14025630832175287055 0.10184458323584141826 -0.31199615192668761487 0.25042912039007003688 -0.013514908916834019251 -0.24173116191865434699 -0.27766326265260182771 -0.2076956707622863163 0.48814417589634834238 0.38005376057556833347 0.5242744943304096239 -0.054265273762584699535 0.79179805653125523435 0.34482387831949040669 0.084562735905970451422 -0.83808883873864925551;-0.13636378189629280722 0.11432694869823840922 -0.20903493897446120964 0.33393002456243336962 0.016335447259569702005 -0.40354280941855552589 -0.070599114204938362427 -0.58435229969923097038 0.19581507332272748978 -0.070726957134204043731 0.23825737453825304724 -0.0032242313799543628229 -0.19877437354683011317 -9.5496496546205745506e-05 0.1824732637274632252 -0.42511936516637921502 -0.18036234140151824468 -0.21622236563554750322 -0.36015962469341955554 0.25113222849355737454;-0.11537233530661535064 0.071749992678062812779 0.17957695391061426204 -0.40640983513334322241 0.090566203686413160323 -0.18293952859361278795 0.20159884893538829176 -0.070861096386084940213 0.003290128588806708488 -0.29642308419907709149 -0.63642723127852496301 0.10767599762507550076 -0.4611788540353130017 -0.11504066487201373981 -0.31815040469739430673 -0.020463107230516600077 -0.04961462890901848638 -0.58354972168125374132 0.25141964598191546276 0.026904438667029982601;0.16716026030008615533 -0.29526289077734113597 -0.21458146555195420357 -0.038124906966166965949 -0.20389772966663680109 0.25469987308548913685 -0.24135045518430747746 0.27384004287009322542 0.19238008613986984363 -0.35976292007989640886 -0.25361226660511582143 -0.26577286262853461185 -0.15627781653174113718 0.26058562546257063586 -0.096236351693889873871 -0.34580338929136134052 0.31529852334576358164 -0.10642411638980153743 -0.40443068504697177179 -0.14756076521888042419;-0.083530812649236799161 0.24591501962408163218 0.24038321230483014612 0.39166004464029297027 0.04564414554301356175 -0.11869845515148969017 -0.25614883744749261663 0.33816254751815139068 0.14650739348322491806 0.29457518053007386571 0.25945354407959042575 0.014683706314133116583 0.27460043716919779744 0.043177316536016002724 0.51181200487720857062 0.13041311051826562362 0.35436670927156255217 0.34666938795529445017 0.17225191524825581024 -0.47756612283885035986;0.14883025394023358112 0.12346186068403353087 0.14217511884134537148 -0.44857719676506169204 0.047562099332950670549 0.098475332884676580747 -0.3218085942159431001 0.26106599469413421799 -0.038568930995681038887 -0.082346450120212966173 0.016458345520509721355 -0.095308606778231899637 0.24396996315319957604 0.077944678721823812406 -0.25677527524784160562 -0.1281533653470643408 0.26560511137386189073 -0.32257030720264456836 0.26652406976708464814 -0.095681971522155129239;-0.13307754050504991272 0.24591165405431789082 0.011938563835525266288 0.47475068098980222064 0.22110417333631279813 0.0090804177252324751179 0.23552496202313585871 -0.11676459803941643956 -0.16748324412736170785 -0.36354153700320279707 -0.17874354352818350167 0.20625812438208124266 -0.33576634334876159782 -0.20477254927103435977 -0.21442913442985839234 0.0057869594802534563005 -0.11681443114710673981 -0.40044280913633351071 0.046168675531996661299 0.65336566905142690054;-0.15642406463837008057 0.10911827273770101365 -0.31569590530310154275 0.45770712398209711935 0.09988894402001599615 0.09190928380836611844 0.81266422337614119265 -0.57026493124386068434 -0.40423899239136196204 0.48414515171848360175 0.49791351240213554963 0.099059829561123774466 -0.083563260447471199299 -0.18128046081986859406 0.67426248367720387211 -0.092418433239016689607 -0.42385679904544371066 -0.072930019015809560701 -0.28022703657980618797 -0.18402664776397884006;-0.51724661108730274339 0.23303436962658358178 -0.32504714506744919467 0.32045005871022547161 0.65016894776072120532 0.32813802946549658346 0.10259180611867954602 -0.29856738373834335043 -0.45705665931172850813 0.41189290678343337682 0.53792750385283438774 0.61165824279884706272 0.20250415342634150995 -0.53419177275841400743 -0.68940428537059661451 0.37594453653687665007 -0.21688472493158403309 0.18588366883380522121 -0.41700089138255730381 0.31175821644496520824;-0.4768011495824314161 0.56078048471593944235 -0.18127931550891521195 0.44904614782912738447 0.50074129625519581133 -0.26668973972069420775 -0.09648418007074917313 -0.41283798596651738455 -0.22860951307829374679 -0.18734640100218002967 0.70863996679004459534 0.54479603465934611783 0.14563586656936094998 -0.50127692362083775546 -0.4222810401256424373 -0.15277296945739540779 -0.38327878686669292208 -0.056217027248586570742 0.22718182844036621804 0.3266271303860822095;0.46038483620664255591 -0.16791877465990068252 -0.44504881029269510728 -0.13135460240629648343 -0.52095057451490367484 -0.46138061417758807758 -0.24624313143095979961 0.27262828667576777253 0.24376471356080398056 -0.22175337651087892454 -0.42904291397490712523 -0.56853218951684802995 -0.062317592004599631217 0.4955334194999374553 0.35464011334978290391 0.44170266575840322343 0.22063729452564462274 -0.25183810306700099568 0.49649447310285832469 -0.34569400386370452427;-0.25264783846868565931 0.11807775672759299179 0.023557354641821171698 -0.36588615969726340227 0.30843138767149591573 0.036120568494882647481 0.56745339292861174396 -0.36949917588855268891 -0.33778790738019637274 -0.31061911588638674298 -0.030105385723233644979 0.31402483386292562306 -0.23064803569460265442 -0.34355502861546965976 -0.94982640209012025867 -0.08178118095849297442 -0.16191460567207535681 -0.25961941354702700036 -0.099509908534820903925 -0.4490058511716719658;0.24332398350784600893 -0.39005649880656723116 0.099938974967393998106 -0.34069344864922745941 -0.24862552088871209266 0.078495103662269138933 -0.10950202810682842236 -0.26025747511174174287 0.23551481606503138488 0.3428914974760476353 0.018635669482713460021 -0.26067741278666156646 -0.2282523603407752566 0.20642993610670770654 0.4648777510044457828 0.133314423920760039 0.10329253045833261537 -0.038113238797156244464 0.088292427528577438478 0.37833517552830586705;-0.47099477825980112788 0.4243016739477041277 -0.46346949440714485524 0.33915601083963775597 0.53979667151443366802 0.48973463887478685397 0.42262058847138150863 -0.37950975737173214819 -0.36313603747089473472 0.21293946437070954136 0.6839681957048857619 0.59427607250513858617 0.37347877795423428671 -0.50638854066809257937 -0.59128060667015702734 -0.50651561917193610451 -0.54583238715251392037 -0.25001531075044358676 0.47371606901420854063 0.41691776928019030901;0.34146859507968313485 -0.20090661063766282468 -0.095412829078404134964 -0.13518200448452857843 -0.47893018361974004637 -0.20278978300351566122 -0.19726217432649156835 -0.31493722701912868489 0.5194363013937657314 -0.36395464054613857474 -0.46565806461998687782 -0.53111452343629461659 -0.13310230027070221737 0.4911217842424684954 0.50010744744132840101 -0.21955294105764400214 0.21137411238325706786 -0.34519967635350945478 -0.25708873116914587875 -0.048963292520054220447;-0.021640110238099061896 0.049229002131728240299 -0.025126617579646230716 -0.18823504515106848078 0.14129535362683390276 0.046600495455787045718 0.30754593312833555796 -0.37022379629103141507 -0.18129448072822929028 0.040248837286070322794 0.6402994108333052159 0.14625486911301383008 -0.065091228388220873091 -0.17099165250644374203 0.38963433592380319892 -0.038494133099065928894 -0.37640160718520077054 0.58347252845292740808 -0.066942306795135908537 0.47000710525745581503;-0.31623592140815176554 0.029437240697284591112 0.037575215308716797269 -0.21512364043366830058 0.2605439652312688259 0.0027183643210765286689 -0.19826590303959812367 -0.16599429229820328313 -0.23674995131555010008 0.53163159331806020269 0.26851371968121529044 0.26223491657649755648 -0.17693990126985939537 -0.27343165273032410401 -0.45896723259608601664 0.029504462322590449541 -0.10030437531953270702 -0.14174793353943709873 -0.0047999277776896538336 -0.52968031118997938034;-0.079708389235926865868 -0.075019555545461663604 -0.065653672791560979682 0.80216125719486619516 0.34989046144495944413 0.1524554546282886891 0.3684117320145932073 -0.38985565576417485767 -0.46069591038201640476 0.50379628693978817378 0.22134990463210638123 0.19444640622450004575 -0.057523738523689520175 -0.10296412218744561973 -0.13546928315325887371 -0.010796191275355236996 -0.25427443042321684619 0.37984548100048448616 0.035470108112479525886 0.39141206181535070918;-0.079982478295884298913 -0.0034413322989171194269 -0.17033765665127603706 -0.059544320943089361353 0.018496802719977556428 -0.06351688988054132734 0.066461910678016433707 -0.46246178569028839611 0.047968388020933817872 0.0020854786549143224106 0.35883258795818900877 0.20775544085897554303 0.43521508021746668771 -0.11949504329557283921 -0.038519135210836830119 0.139614579579128123 -0.67906481764434201143 0.19905391021154064668 -0.15492240165196902169 0.41462945619233332328;-0.5205286358349991982 0.15928712832001004629 0.2881288711247776968 0.042517249668013727681 0.64546427668571826342 0.31589175693193288774 0.20289570596010639281 -0.34192075529397614186 -0.3583845920679544772 0.55208819765126448953 0.22088248374050392653 0.65610637072882604937 0.12442494110917076688 -0.58896717188606273474 -0.51865114950429735963 0.29516088429879810962 -0.32351675493671377959 0.20003962588378410015 0.34259771223775792315 0.19315807303553650387;0.27758250703189618624 0.0048911386640802271761 0.3347060458280702866 -0.23352943279727791936 -0.31306212365319879831 -0.4001623513616158756 -0.23351885775186465022 0.14970576673749999563 0.18202076442059936245 0.098547821844896660304 -0.10419130515136489079 -0.33596192530682206234 -0.025074938527019981216 0.26906819263988812629 -0.10355539757929384248 0.2242136446567939978 0.20462906958350934872 -0.25747042081200011143 0.21325047180279363412 0.22266032717801079177;0.25727580347364953361 -0.37137635053997758439 -0.12821565803540771467 -0.044397488757195430198 -0.16397635524795284301 -0.070843743190786304109 0.034585745598500618192 0.035814286795003112684 0.3687696110459225407 -0.53611125979048512846 -0.16617283406748922792 -0.22970983720799101335 -0.28996185111220790231 0.26515626632234878812 0.46353284018247875586 -0.19270149593935351207 0.37004638156348818034 -0.53458732387132790187 -0.27886702479212988059 -0.33247484439388080846;0.1527414346886607488 0.043120865504436334259 -0.083489737196383917439 0.044705378980278424883 -0.22757247614960343651 0.19761074822949362129 0.0017917578253438619894 0.35900709556417675916 0.080615974181420635425 0.08147938069118776383 -0.1792862289604918169 -0.20393417297671076471 -0.092196755351492132302 0.087912660195456918122 0.11384698494807808655 0.024056765416804012231 -0.12792382389239731477 -0.1183587151917199487 -0.4517263150321969567 0.16073815588580900315;0.46635221121988534865 -0.41788183528926087229 0.35178955008057627429 -0.43308773862065619298 -0.52604569549439594844 0.34917900889133340891 -0.055877064218727554024 0.30341532471121807069 0.37845998523258478352 -0.0012476619563381269144 -0.53610315471245251207 -0.51813231400148007655 0.028540619626648191381 0.45521316124398214509 0.49517270620201447873 0.3927567284996258401 0.3553127575920086989 0.16733772780327355045 0.36817531276181791888 -0.19766032076090628289;-0.46090369899539879173 -0.10954712983250239344 -0.38134377018422210837 0.10971027999077856419 0.54890561088966594028 0.4349096131045470659 -0.090928313859054962776 -0.032326334229416858745 -0.29863154664858876375 0.039322014072778840865 0.25160389008777284126 0.51646140390960204591 0.18766857378109463861 -0.40781618926486934429 -0.33902925432159891628 -0.42726904048908598277 -0.032106681708295913003 0.54739576357187580413 -0.37026061672258275825 -0.27934247480574381228;-0.32065252243335323756 0.15570204700147627364 0.017863023047508096858 0.54014395669676040779 0.29140174690665732182 -0.057086540234217504619 -0.3905543596313032495 0.091196057041677408561 -0.11972308829780871353 0.41695011828742772897 0.038503794550153125353 0.28087944598798553208 -0.2941614369078632274 -0.21839352802084080474 0.17225273787862807606 0.047737723098227505258 -0.058133253057159772759 -0.12414386943058354495 0.12055410288487541925 0.51691628269270206708;0.14323533762666595948 -0.031574266501010646302 -0.01509859760444716946 -0.009221121636627180318 0.065712428039877646135 0.29954381494656578955 -0.16361858012982910537 -0.073155296381270495942 -0.18632313648437467957 -0.026153451580409780436 -0.045400197605885615759 -0.0010767616346993747357 -0.28014163950774290113 -0.11813318014751923724 0.028098896232494063874 0.19762153956619599171 -0.14603794931591598805 0.016998983584212475767 -0.055753260312976893331 0.011877265259238815875;0.56963966691527134767 -0.16379008604473210831 0.038371785601180839753 -0.38996276998927859037 -0.64428529087676600362 -0.046189856839789898701 -0.54500478581611766504 0.69997392753213727445 0.72633941221246656195 -0.55286457902160979216 0.058363488247032635492 -0.64007261628186951086 0.3215550779127073211 0.6594411521318256364 1.704790188062451195 0.08156033357326719635 0.26839382968729336154 0.3162087761626515281 0.10634857867740389603 -0.11301893425856567033];

% Layer 3
b3 = 0.56876389546536976205;
LW3_2 = [-1.307910649738810438 1.0533325697809685639 0.73496609692241188228 0.73303408732804065107 -0.63883316149919000537 1.499648116958161248 1.5407800382457812471 2.0981833422552464974 0.59212126146258470261 0.56923053293423708787 0.9645084024881125373 2.0187284610169875521 -0.62622865829675433691 0.70195822959540354891 -1.0353363870984286788 1.330228666968435336 1.1041578270930005878 1.0964473520738999834 -1.5610700681325639927 0.4639847923017310638 -2.187516208737541934 0.93634442632427050679 1.5977076205281603993 -1.4916558333981755347 1.2626943575187405955 1.0874956376557312066 -1.3286802369055934481 1.3754310513908205582 -1.4507621863961108755 0.64800500313659004092 1.0545889238569656765 -2.4112303778545407518 -0.83887704946602070777 -0.88310816430619953898 -0.64579201919301487678 2.0456818317656288997 -0.99187590247466073112 -1.6326745549374761524 -0.48090532161831855973 -2.0419774971456554624];

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
