function [y1] = T202020_1(x1)
%T202020_1 neural network simulation function.
%
% Auto-generated by MATLAB, 24-Feb-2020 16:37:44.
% 
% [y1] = T202020_1(x1) takes these arguments:
%   x = 8xQ matrix, input #1
% and returns:
%   y = 1xQ matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-3.22336;-3.058485;-3.251364;-5.629747;-7.902363;-7.630164;-1;-1];
x1_step1.gain = [0.310235282438201;0.316868377153853;0.307563225772322;0.186375103939066;0.135346064660906;0.13424887486018;1;1];
x1_step1.ymin = -1;

% Layer 1
b1 = [-0.26858419268181304984;-0.34137159439105285719;0.23295785621901834994;0.22089581981271994016;0.17951044202462496102;0.5331047401686821452;-0.905610656578004658;0.28573476180019552473;-0.10300539560332193367;-0.18608297009731061822;0.77115828580229683897;-0.073069933385926960634;0.21658048156250170324;-0.70216415521579800263;-1.1298844128598346526;0.24227649524177433116;0.28911429979890301833;-0.28119111578494232484;0.047021334881539818273;-0.14699979980581301398];
IW1_1 = [1.9124030405305638691 0.43878544242710315615 0.80292947326474084502 0.34182654139922030723 0.56077843773586677489 0.52997724099865684 0.18456833321529847458 0.010856852873980875687;1.1083320539979109043 1.4942448652301487666 0.88923105033099969496 0.0746257752209194547 -0.15194503872523704158 0.11837094111805157581 -0.72634508247264240044 0.067371394901183948023;1.7567009426444348819 1.9270788000237495385 -0.66733989626855549915 -0.52926589945851065799 1.0416201612720237257 1.1012203587066744515 -0.31352591548879638061 0.022982792386761038278;0.54441294427784181487 -0.29155793163303184157 0.091786084227477790498 0.78376434617133194926 -0.14395340989328750392 -0.32404797067543578137 0.14524340125967633019 -0.20887204099596728968;0.42809752183295901595 0.90126132127776859004 0.34164211824333545753 -1.5721503198926267686 -0.5856202342063046018 -0.12974377458369285021 -0.2845171305373430104 0.050601993885135310158;-0.019016831135461948615 -0.028416146407142836572 1.3019619114705580643 -0.35297581050858151119 0.5977275548405707184 0.12895825881515832201 -0.01608824019935911967 0.067817920268565964892;-0.39680946996750304789 -0.081074821504914240666 -1.892953955766855989 0.55418436325740627613 0.25235075038873461262 -0.74319600319461198712 0.071008917680971880837 0.035025005308336701337;1.419290965435119567 -0.49443905603747095867 1.5597053078737803666 -0.31364798755821610143 -0.32681805610289565989 0.93765334135159517093 0.069082696889730746403 -0.059075404391442203067;-1.6507974974161081505 -0.13628534851357582136 -0.936503842992879032 0.21750028812295213454 0.38162898105261766624 -0.01739417435770045442 -0.068043061274434699093 0.00080577073532206000116;-3.3238359646590236451 0.63498309279616949041 0.51627069482888465757 3.4209915802850168198 0.4754078374856147704 -0.43315612780720652575 -0.71989065048712963524 -0.092119289213108121461;-3.3712525654560985267 -0.68889498018947348257 1.2697956149691140837 0.97199480654192083051 -0.86013931548357314139 -0.044059805064166110422 -1.2919639649152006022 -0.027684705040405481469;-1.0618278128131968252 1.4129584880049541606 -1.6492514522817702805 0.5187263949304828925 0.090388660345215254832 0.38141866197983004261 0.1051544161459407184 0.061089321355052808582;5.3500874934482851586 0.96687166764774268302 0.57383764075506327185 -1.2044495495388884976 -0.62352703595367842127 -0.32514337804792653452 -0.13933645311204131967 0.048600936632732759379;0.85306021775107387928 0.040798544226290568704 -1.447214575573376738 0.67763192600895438833 0.058914587406485079135 -0.097908629280956366303 0.10506548309982477463 -0.034628824247099530831;0.12149382865396474163 0.34188160285693142137 1.9553420707226252251 0.051219614526571914659 0.28218566554990692774 0.77931145385520694813 0.035733665186300965499 0.013488582415320365601;0.95476728654573839172 -0.1865784899282404985 -0.80922784587073792739 0.39908220492962004355 0.024240939877779737327 -0.00031522026019113031335 0.12856849917595822297 -0.013363394154668726405;-1.5568427589815994061 1.0374198198915489666 -0.73787876610813241118 0.26233396568055800113 -1.0082300109844781755 0.65937341048404607413 0.048665430146918584042 0.0028132894440520703552;1.3086328144911405502 0.51259076543187098718 -0.38914296698567585331 1.846870558589406075 -0.84662862080373257889 -0.63152092234557999362 0.17349282755542413947 -0.0022295701458641642261;-1.8481212766651835544 1.0500920775678448393 0.098816485922207847326 1.5621331881305862144 -1.0659737550806858142 -0.72202858600583164073 0.15509270833130728295 0.097136640796075701676;1.716792394796435417 0.018583124113862821425 0.46172307595535266866 0.52964406065227287446 -0.021936313092977915984 0.010922014900441029753 0.73112823973677032452 0.011474514584640368997];

% Layer 2
b2 = [0.004219416690180976745;0.39034144023024314185;-0.10980996822179292516;0.29311066175546707235;0.28022522510327957379;-0.37460656823904237012;-0.65318099403459328745;-0.65453377246459998329;-1.0449242794671376178;-0.1835009848938983501;-0.15798141402735477179;0.038504374963637726104;0.31542672776405483726;0.56727719806689325122;0.052581019229101542967;0.6005115129207190261;0.27875686777656116799;-0.3127451826837828186;-0.2083153584716591622;-0.32328236143349303733];
LW2_1 = [-0.26657202616791386696 -0.20018690225403515393 0.18118433446338366721 0.13882478346795695812 -0.022470395607451700698 0.55274002572352298746 -0.71994102847315832872 -0.0061182503159228334572 -0.12894664522955384678 -0.25942804738968860967 -0.17062961652147123903 -0.35845757779410697408 0.81170011138897091296 0.52431692637963800419 0.034735861113833768621 -0.45311697865225664117 0.79775617842556922898 -0.3169754659790484097 0.069062626222291326195 0.079919042490256095301;0.48488275085644061413 0.05477868032720393382 0.11399557249981891183 -0.071968451770409391788 -0.30380277522524729017 -0.0049615270238471675945 0.17014755369534217966 -0.11247331697861787103 -0.16704243319622255615 -0.080152813636605479974 0.15681095194555735861 -0.29027775123588223094 0.57587307430942225039 -0.031157630453859070513 0.37813175325905312718 1.319647355650103826 0.30591106393612049752 0.20971947770196419647 -0.31853356862215781442 0.054814326580457962967;-0.13921513702890170316 -0.54338704690004335784 0.5558676035281583383 -0.11470568794469215845 -0.027583176540715351882 -0.2159884171561841093 0.14593354212524875524 -0.51218080427583090319 -0.0093125618580916520034 1.0941082814583007465 -0.079760693170108518113 0.81216428839630161463 -0.25415947160928542559 0.0041109976234542534163 -0.3231944057451660024 -0.56319809393815545295 0.4889481026328809099 0.18202740447540630142 0.93957313406987652549 0.14736354861465025823;0.21141219400120420113 0.65664015339101144964 0.25545949243007154417 -0.25735582229018305611 0.91909840267529985258 0.16838897247963657833 -0.20648621341384890138 -0.024973272624196612285 0.37857727963634868473 1.6261382776223534119 0.53365491633327721832 1.9853965827203121641 1.1452538946854329271 1.1010954213404060553 -0.247503110073279603 0.31912984846377423498 1.1429622640121923638 1.4132593501663561764 1.6947472966169341291 -0.051188403466196921721;0.011456005450008410751 -0.44549275012275618701 -0.22756214248319425741 -0.27439696265163382449 0.17421727310838813763 -0.1992193369830605465 0.41482532432767799113 -0.68873717362583297685 0.091363595335629127492 0.14905353281490868467 -0.21690299677405486256 0.60784489013170228677 -0.51580063336557968601 -0.02254950027790201747 -0.18262224283780187362 0.62663885777440508917 -0.35504044080549551632 0.22819420865076078808 -0.18190850388002058335 0.015238396443782204967;-0.41694971483222009745 -0.45334927870763314095 0.88302972492274534044 0.29725778868439067804 0.23893198649839653136 -0.086002312603124478341 -0.21802353063787088816 -0.054615721787528055675 0.19408139618434611107 -0.50389528011680573272 -0.047777689819053878462 -0.026351282739735124028 0.13080582044294475352 0.30036037754137084876 -0.15487160442572292518 -0.46332821100108428913 0.61788801680924565485 -0.14211769463881190889 0.12420009483459026889 0.10627261305380929224;-0.2837239748926536631 -0.018593976749340924287 0.029439416798120683627 0.1409837909259005484 -0.076670647713709003113 -0.12688484100438329749 0.46366613233427916851 0.44978158639732446211 -0.01395605388720209776 -0.012512272874686180754 0.051912117082649845978 -0.11931991204614388058 -0.29269356411425762055 0.13016703950224992048 -0.32298324744870432479 -0.5865172695894566024 -0.044454574885547416818 -0.061849475489617916635 -0.0053919535761434156385 -0.051025555868083181943;0.14002711742037368681 -0.027132628372176545439 0.32338497180024378119 -0.33450982033025039586 -0.11667080043289689006 0.039998201596197442698 -0.21166076886147452973 0.43229918773336328819 -0.086203245105163708861 0.34596283616468653799 -0.62396516368121002216 0.040190229137138815119 -0.49840935256603086101 0.83166896602482720535 -0.46312427916415987994 0.23346328832817284105 -0.18408219967368627534 0.28255957904039058493 0.13155148663019067268 -0.15908615051215535541;0.22764769830188799626 -0.030566435206095234067 0.10812486379262244152 -0.12680592641116969488 -0.015697135362818422089 0.43128692037461630049 -0.88581745647665055543 -0.31969614654001393683 0.39849190300044556867 0.02189697970812444841 -0.0013203681908111782051 0.28441163766658650314 -0.22179946493205798808 0.49831268098998621197 0.13355913211371472915 -0.48990793093962464733 -0.1030575635636722942 0.069833331236123147301 -0.067120960276123742272 0.20653679975445515815;-0.86729413126092314013 -0.11578260213037359916 -0.019635078861598913996 -0.0051374321579282199168 -0.71040457181161065225 -0.56741109705123771167 -0.9776615829582407935 -0.18395561968678209031 -0.45455378530564105599 -0.19451631641579741894 -0.20096563279206169916 -0.053888514064413715887 -0.33752551444997136443 -0.40753233294339735737 0.51436203724337692389 0.26099688794516667789 0.049767285556110214817 -0.47924398185289474039 0.15670591960590593561 0.070204802566247184736;-0.038445651367793105824 0.19326934100748252576 -0.09342550619290890257 -0.01689957587085335472 0.5850268685170748606 -0.54659949919458439282 -0.049800044457230727146 -0.22285037535677304499 0.31459435867576651669 0.60771528174716382686 0.81807292423424982086 -0.33605650853357560415 0.29847634749826285061 -0.48401129134099063478 0.046673667925570469506 -0.17111573824404890942 1.2081217769034013898 0.48255157294866218853 0.72081251815206293099 0.02142205766468102332;0.28839180178631945717 1.1099430342682703454 0.67838478653503386795 -0.15817761865954407807 0.34498913964323452142 0.067164804560584864812 0.051585932684395020831 -0.42290557070569584353 0.64882917525975813966 2.0618426076013678383 -0.17004190555131112883 0.16947844711291021325 0.69974689885154595093 -1.5909757858007587128 -0.18061034420723171001 0.073197287458467777399 0.098157609407983723027 0.79355981953038468024 0.38539338098694125856 0.16996271169895979525;-0.14759115085777277376 0.050822662463741716254 -1.3736645652098853354 -0.47772898050444795315 -0.94555908192696358494 -0.29514418585737350886 -0.36388715984528913205 -0.66155991906825728499 0.24954163053566941199 1.4773827027226444919 0.30199023676366826274 -0.37178272149534208335 -1.7242572124985187099 0.40778376826104384412 -0.60814666477060075334 -0.15998729279503448586 -0.20358248221182465021 -0.13358693416448605418 0.31710151271274145124 -0.15827513088109004813;-0.34075312455016820712 0.31848464245138147222 -0.021166047841405778718 -0.29691430176064020907 -0.58977001027428288538 1.1421072725075052112 -0.1904083819860662441 0.35653183318846848326 -0.42977696239566864911 -0.04831014758950769733 0.075479049826711760995 -0.1548558598328438618 1.2596972668258978878 1.7465423786136577977 0.68906757344009705868 0.11764991255099711831 0.30532381091622301517 0.73457620691469882601 -0.66611718937209019042 0.60274363750223569625;0.82880247347937485713 -0.028272877769267127507 0.25834017458349439744 -0.044455726917697072076 -0.54991507401566996904 -0.03609296897037306967 -0.6876879626236704901 0.072213354650632374798 -0.96110159001902228937 -0.39356529859302541441 0.067364040246934681067 -0.3511849594283009135 0.72351702188728161946 1.0095025723162684983 -0.2346833367097519718 -0.47367948699014328362 -0.43680571095194886677 -0.4026315506490288354 -0.31251773447773029124 -0.21842091174720634772;0.13414424625730356211 -0.1065592408719811951 0.73671600379816359894 -0.19354159639200477216 -0.052096064685491223767 0.045939702288443656775 0.36454608301847962748 -0.31949427561717103075 0.47845716415140626676 0.10120500844438244659 -0.17507580617681628432 0.39364357740736038993 -0.45762563534323891234 -0.092524872699318666536 0.29100212704610778358 0.66194360291316367562 -0.0028173507634663298022 0.23241286587447129719 -0.21887270479022452596 0.041865939974421104153;0.50089844506155423165 -0.37018105455799865666 -0.20967182515826754408 -0.15871838814362110304 0.23123233500112175265 -0.64472030303723082412 0.28514654463103517035 -0.033267776179301364325 0.033327039105855660317 -0.0050875409177876459998 -0.070592524495226349757 0.38818699442374671182 -0.46861475156843618306 0.80833342814799180598 -1.5010520786375725066 0.08365046743328244383 -0.27690868204260976615 -0.054162769332959027901 0.5051410232889376184 -0.42946333930446300986;-0.92242515819237913544 -0.44778986434723805354 -0.36309156765834571523 0.45553218990280341938 0.97069054355495143493 -0.46472348298285459656 -0.14455480759332162144 -0.92924095350183211206 0.63512615598764166069 0.026783339376803657478 -0.5008393345973923827 0.84401038642761860853 -2.2471440210748774291 -0.65151056008769669958 0.19046101841701637558 -1.0202191366499824277 0.23299923681688600174 -0.99639570378254194161 1.1908050578964206601 -0.68691595916620551421;-0.59581671619330767165 -0.67369361897508173165 -1.0615048150523287429 0.041574634197549927006 1.3322544268713631954 -0.22085962239300177545 0.31886096619302911614 0.059873320009045626067 0.62726760099227829581 -0.20614772996036948016 0.70089459503964990361 2.2274196000694765907 -0.77675159424624418136 0.57148370771268852675 0.12152705124970211714 0.052809879470978732252 0.14039099045042674474 -0.3680418108112819553 0.4520606575562231555 -0.92696387423650317139;-0.090634223093089855139 0.11186548944812636786 -0.076091103518435940223 0.0048946981669156699318 -0.13841716103038217978 0.058414714825502117579 0.21565916773591037314 -0.0038820656569631207047 -0.4746491225914317158 -0.0075282573723563801088 -0.05088267360386902749 -0.23244685033233636329 0.21360140417276146652 0.16609151716557510325 0.43530482193244951494 0.76800715920706641437 0.10593522823576932701 -0.00087453477756956933282 -0.053507174708158321275 0.07422974794362516282];

% Layer 3
b3 = [0.3705483529523483921;-0.033478051345311891918;0.38449365207239427633;0.12168751221361824577;0.3895239255668929701;-0.18973942228812534783;0.10575719184706074139;0.20106962755887738958;0.42834680547218284552;-0.046370004381150735362;-0.46459591214785644908;0.29707926599355155561;-0.017602875905723777694;0.25403600437342654939;-0.28992322638869932883;0.37807307679690449476;-0.19728109179881148094;-0.20604028829459009042;-0.18107343198304931553;-0.044476426292799721429];
LW3_2 = [0.0017077400970438019398 0.022295336495427191259 0.29796774556195104733 0.030202268719734214064 -0.17780552945799413256 0.22839981568279907331 -0.093358186533911299843 -0.083600162226887056449 0.015692986988787644559 -0.12574346922635928903 0.19217439868791791335 0.19547500648144397473 -0.36395019427389430078 -0.027210723952856404051 0.42272146778061431283 0.13223894230589555709 -0.19326698117876983662 -0.12646372147135628849 0.20825782906185041909 0.46840249332140115879;0.037029438944878992346 -0.78797791587765853016 -0.27340815578750188308 -0.04462019018935214909 -0.055218530087482321778 -0.026159606147162856732 0.41554987662757819633 0.32538446527082981463 0.031296443786014843669 -0.72174263091954093952 0.32167739314685311891 -0.072250948843196202698 0.097345102320667092899 -0.0088572963392549823591 -0.21318289815003801935 -0.025432203031148205991 0.54139068383773536475 0.38241230210499455033 0.19948223128484540712 -0.38164640849951264601;-0.21214993372370641311 -0.49126397844836550144 0.51634824830315795285 0.51627333326905122668 -0.30666375772713083681 0.21093164551508244986 0.13159593872480665477 -0.85063423974203433708 0.65249167501318705575 0.14960007779723497046 -0.3201327475433638492 0.22763644034096305879 0.44027699472019488081 0.035083404897326656013 0.42468148669343147583 0.31328222959089085942 0.030542167165635663445 0.75211880592835234083 0.17764405623902598808 0.073910700217030736447;-0.11896412886770701967 -0.25278750355027185259 -0.60694009418824046431 -0.31882703587320643246 0.052426523948823453014 -0.20759332650841103018 0.1171236000973842617 0.13896828772551789166 -0.44277218745639485009 -0.45782823688421392383 -0.0026497616402547069341 -0.28225951257952047735 0.46231082550825225441 -0.31636948273999204728 -0.27566492898199185246 -0.095093383049845908728 -0.14112227528543866417 -0.43186720347106344331 0.34798898447042481141 -0.38216395731058816487;-0.12835723146739044798 -0.40400684664648001121 0.25195601652370086532 1.0210446064180396242 0.011549373293110530569 0.25446649449971386892 0.2523250857027348748 -0.22628264317651919368 0.85349408592907205495 -0.97804722548883149535 0.25932488517272095363 0.36376111127575200266 0.64855860529927700231 -1.0289934408694894064 -1.0132663847362166631 0.48003469796954417737 -1.0213391027924854093 0.57466409491582193247 -0.64474443947259429954 -0.38823055696801761494;0.013026926257301930934 -0.17774540320157838913 0.35333278182508609522 0.61049833780440077025 -0.33558412796017184077 -0.097502676590759923547 0.19228711791711566859 -0.071579707607878073627 -0.26438305020081026564 0.30054603731007573098 0.17843654350964099908 0.76039968160166593325 0.93906195191649444176 -0.67041190920663007446 0.066280357961487373908 0.17796155920835535214 0.14107600565989505959 0.36890137057373512119 -0.12961063158999219791 -0.083472318257331909463;0.40763397337382001329 0.47587134899252103715 -0.071273528672037428366 0.23719559858337235791 0.45176820856936422688 -0.13766488443283939058 0.50250334091305759454 0.012329336441644500252 0.066068349988091357172 -0.79280691217850607355 -0.14707597455493998106 -0.40733910959630359461 0.57915953140731091242 -0.35951064782155506272 0.12513697598941542211 -0.31708936329512837915 0.025640494922148079904 -0.43162348493010277428 0.1360848394687330376 -0.60647676972337039025;-0.43048982054243528639 -0.17967264517136513624 -0.66344542539752882071 0.99154739714085737834 0.13902267363735218431 0.41704409023153826697 -0.51228148202501899711 0.11074352340214915769 0.28120852167509374775 -0.60173554851804722521 -0.31374013874578554617 0.16253171139961231084 -0.57394613025799801065 1.3416621834916953038 0.41326282768549349456 -0.18057814180752765343 0.033251058935601561506 0.27497668568947447598 -0.50843415287958149129 0.2412524770728417256;-0.034948711573408314246 -0.049485825127299556958 -0.62248301536535655121 -0.40558347507672687371 -0.10780684357081352087 -0.31098917985628043992 0.40366072213004816627 0.032352719714448134603 -0.036959932858811654632 -0.30376710097598302873 -0.03912172337341373457 -0.1319620047306682975 0.55453951361899167516 -0.17353616675953686954 0.097566767713322552491 -0.081401780953091376625 0.050933697765508642585 0.44211233041082792505 -0.22901101928051836865 -0.058931831462490084916;-0.25877508162545942438 -0.12895039267112776149 -0.17143471984960265142 0.04323768185107224743 -0.15875628024610904943 -0.12638478178586665113 0.44652850064204979708 0.0076205480367040282635 0.048040797312790269047 -0.36693274818349780331 -0.069733870971427636132 0.23473879401420533197 0.20917766369978565977 0.19554631734043323443 -0.14249394783829660849 0.38570079630457410058 0.32211317758716734305 -0.29968379172469494787 -0.0038759628060244166389 -0.29885605944336829376;-0.18903397769520563498 -0.14247680168942689072 -0.37938320153042676131 0.57788522273976838939 -0.27089279238140490103 0.097110264873813634257 0.2032604023706551799 -0.14918329850037806428 -0.57443217818672109853 -0.516364294639970689 -0.010485529350825214742 -0.59951918765714562465 -0.089776971526028301329 1.2336215937729453351 0.32018581109819715946 0.47835942351028409858 -0.73710234154403153628 -0.02213189302541029857 0.33301940095153526045 0.39414129975656564397;-0.16521860592787779298 -0.43619988354803945763 -0.2483250484116164003 -0.01187569539439400565 -0.07979933156958711471 0.37012339257056492414 -0.43472638753488612728 0.20396964235994122228 0.27940422137116038837 0.10249321155381581339 0.3165776733553054556 0.018781719358765849165 -0.33744922015473477472 0.26800553811930183379 -0.077382122764642999324 0.10307421880653042101 0.24753827926314808661 0.11972889236137984614 -0.053476315354273357139 0.021607216797267653208;-0.36622348617932737325 -0.39289621787278283804 0.44410534487437497431 0.53658505787905452777 -0.4690296967466707212 0.53051945161859437317 -0.5559058315446873122 -0.36363675334263190209 0.3153487140180182835 -0.32309366420884888527 -0.38349259270751284712 -0.040905933422980392256 0.51721677257340492595 -0.65199245968556940323 -0.61207132569962918733 0.41846762903927336996 -0.11243070841643822144 0.021131400995172026219 -0.030672003668102038737 -0.49970645857462486772;-0.14960810398240820263 -0.027361100752398170854 -0.72441134183961231052 -0.49545411780980480598 -0.1427481735790698536 -0.21135011759861804803 0.13355599923798361361 0.058231889088885449568 -0.059670628288395442806 -0.38780646989766692778 -0.23443147235940925421 -0.15103363063651156772 0.6674460389663591231 -0.94048649662828909257 -0.36633100718672650942 -0.037885253534465028469 0.052105603952692307324 -0.35818630584251398563 -0.21612400291751787673 0.22957534025389667032;-0.31247982869329460343 0.29713506271484935173 -0.12849400174327349489 0.019045962175641410752 0.11580189136114472925 0.17234416824418724579 0.47599993137931706499 0.17006911443803801598 -0.25696417130195514655 -0.52400974128948241137 0.085473650742607903963 -0.19970761568017592058 -0.11199462071814332564 0.34268190360589595445 -0.047975412603255607191 0.2418479616237085017 0.50076029674948419057 0.8393391439278431454 0.30333061699605712302 0.11237013013195190636;0.2746438837575650993 -0.35249377208307103304 0.25446562640631498198 -0.0071575908422872765169 0.13754406867274832083 -0.065348424459809484333 -0.016278466332193326782 -0.099319926661692578551 -0.045869660226828722072 0.025234841925223180231 -0.018071337981288980445 -0.40445317476139952539 -0.28660069095599488564 0.44704561258159475345 0.71337199359800140819 -0.19338568635749009417 -0.26980131282991226538 0.85812074505605151487 0.20423182733013151013 0.20922094958124748398;-0.078100672983920343961 0.3074889326940676848 0.071520210655854288273 -0.18469550471241963363 0.15300151548145743408 -0.35658313304113448128 0.36401364947315240084 -0.00031520606730589886824 0.29378163972560272788 -0.36806231985441884014 -0.21566292826752966882 -0.089974931552072798246 -0.31612902153396466032 0.21164379188149506916 0.89060735537608948231 -0.3929199062051855762 0.13457706873675995274 0.079879081890210548722 -0.11914055597498959815 -0.27013753131418488662;0.05335387598414711996 -0.25922701963592870156 -0.06519492598783625803 0.18653923678509723438 0.25595195867686898472 0.21092327040973407604 -0.71080951373501144452 0.19672710353366795744 -0.48838007088253232446 -0.083757841288311402073 -0.0012497248816573857874 0.10853903815508611497 -0.10773733421126263643 -0.19081770730959710103 0.31574301009332511603 -0.1269634139216047064 -0.20215365286535258438 0.56625232779978551534 0.20051913614437127942 -0.39671708263713773013;-0.16977707984054787871 0.063064231145732757566 -0.22453096769486921569 -0.05701035934783839354 -0.26163119837386161759 -0.23014792166291464515 -0.2324135928970583187 -0.27012710973988246499 -0.1316682322719319731 0.18773228638248323774 0.33815418800664837073 -0.24072867687936413961 0.062881022741495914974 0.22840776056915332304 -0.38419600946822568543 0.37407216299146894833 0.046913966951210007983 -0.1326061629489069249 -0.073122390621440783165 -0.21839023666266335622;-0.14229972480131344859 -0.29236194008150667711 -0.34764388356935060909 -0.18579576710857531596 0.030021347166445073312 -0.2077317111800494609 0.30980838287479195436 -0.12584207779346665013 -0.21245810560982830717 0.21129604478774990928 -0.13118219943440237452 -0.23682761009517708239 0.27353256834930966823 -0.24236323590008612694 -0.035428753090161489714 0.10106846949777413547 -0.005213180029839985935 0.089868098590675168924 0.33853578998521993704 0.52496333967977293344];

% Layer 4
b4 = 0.51960929087880303179;
LW4_3 = [0.76886803738444964296 0.69005708516390695806 0.63494031610568346657 -0.75547321537140565972 0.11307190252450252665 -0.20929524701423468658 0.60413846722064634687 0.18142502396582238511 0.97548378238641209048 1.2458133736721752616 -0.24649880378560118377 0.93210655019189336823 -0.78185784254562984419 -0.47356148431197930826 -0.71306382496980069696 0.91594688074627406138 -0.90159351550921273954 -1.595026591618014189 -0.90512771318475326865 1.0050668547316963242];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 1.13909747029234;
y1_step1.xoffset = 0;

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
a3 = tansig_apply(repmat(b3,1,Q) + LW3_2*a2);

% Layer 4
a4 = repmat(b4,1,Q) + LW4_3*a3;

% Output 1
y1 = mapminmax_reverse(a4,y1_step1);
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
