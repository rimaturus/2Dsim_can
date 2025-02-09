#include <stdio.h>
#include <math.h>
#include <allegro.h>

typedef struct {
	float x;
	float y;
} waypoint;

waypoint trajectory[20]; // trajectory


typedef struct {
	float x;
	float y;
	int color;
} cone;

int giallo;
int blu;

const int numPoints = 2038;
float scaleFactor;

void trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory);

int main(){
	
	allegro_init(); // initialize graphics data structures
	install_keyboard(); // initialize keyboard
	set_color_depth(32); // set the color depth to 8 bits for each of the RGB channels and 8 bits for the alpha channel (faster than 24 bit since it is aligned to 32 bits)
	
	int XMAX = 2000, YMAX = 1000; // screen resolution
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XMAX, YMAX, 0, 0);
	clear_to_color(screen, makecol(127, 127, 127)); // clear the screen making all pixels to white

	giallo = makecol(254, 221, 0);
	blu = makecol(46, 103, 248);
	

	cone points[numPoints];

	// Hardcoded points for testing
	/*
	points[0].x = 0;
	points[0].y = 0;
	points[0].color = giallo;

	points[1].x = 0;
	points[1].y = 2;
	points[1].color = blu;

	points[2].x = 2;
	points[2].y = 0.6;
	points[2].color = giallo;

	points[3].x = 1.68;
	points[3].y = 2.35;
	points[3].color = blu;

	points[4].x = 4;
	points[4].y = 1;
	points[4].color = giallo;

	points[5].x = 3;
	points[5].y = 3;
	points[5].color = blu;

	points[6].x = 6;
	points[6].y = 3;
	points[6].color = giallo;

	points[7].x = 4;
	points[7].y = 5;
	points[7].color = blu;

	points[8].x = 7;
	points[8].y = 4;
	points[8].color = giallo;

	points[9].x = 5;
	points[9].y = 6;
	points[9].color = blu;

	scaleFactor = 50.0;
	*/


points[0].x = 237.34316135087343;
points[0].y = 297.0283562073332;
points[0].color = giallo;

points[1].x = 238.01723062811092;
points[1].y = 336.61120628669806;
points[1].color = blu;

points[2].x = 238.7836034398728;
points[2].y = 296.9563114544164;
points[2].color = giallo;

points[3].x = 228.74479492758724;
points[3].y = 333.1506484498885;
points[3].color = blu;

points[4].x = 229.80428215693777;
points[4].y = 292.79394269851787;
points[4].color = giallo;

points[5].x = 219.47235922706363;
points[5].y = 329.6900906130789;
points[5].color = blu;

points[6].x = 220.22612980517792;
points[6].y = 290.5857150644785;
points[6].color = giallo;

points[7].x = 209.7862589073918;
points[7].y = 328.9059531367843;
points[7].color = blu;

points[8].x = 210.42505371675307;
points[8].y = 289.8518534536435;
points[8].color = giallo;

points[9].x = 200.014506433526;
points[9].y = 329.87152506143883;
points[9].color = blu;

points[10].x = 200.56093513666983;
points[10].y = 290.6476377005463;
points[10].color = giallo;

points[11].x = 190.21708591997904;
points[11].y = 328.47007023056534;
points[11].color = blu;

points[12].x = 190.94494409938798;
points[12].y = 288.50099970311175;
points[12].color = giallo;

points[13].x = 181.76979666198076;
points[13].y = 324.739335805983;
points[13].color = blu;

points[14].x = 183.8389750618246;
points[14].y = 286.32364220831;
points[14].color = giallo;

points[15].x = 172.53550129089453;
points[15].y = 321.1782526334378;
points[15].color = blu;

points[16].x = 181.87602802196056;
points[16].y = 282.7101474942146;
points[16].color = giallo;

points[17].x = 163.30120591980827;
points[17].y = 317.6171694608926;
points[17].color = blu;

points[18].x = 173.83554026158995;
points[18].y = 276.94389868453527;
points[18].color = giallo;

points[19].x = 155.03185447857436;
points[19].y = 312.19984982674333;
points[19].color = blu;

points[20].x = 165.96600643585273;
points[20].y = 270.94365036450034;
points[20].color = giallo;

points[21].x = 147.21678206816165;
points[21].y = 306.141695450919;
points[21].color = blu;

points[22].x = 159.1644878781337;
points[22].y = 263.9947231418557;
points[22].color = giallo;

points[23].x = 139.89327165479685;
points[23].y = 299.4843897205513;
points[23].color = blu;

points[24].x = 156.25374189209245;
points[24].y = 254.621283009035;
points[24].color = giallo;

points[25].x = 133.4543987785958;
points[25].y = 292.0350741177135;
points[25].color = blu;

points[26].x = 154.14974793809262;
points[26].y = 244.95114528099018;
points[26].color = giallo;

points[27].x = 127.75417508225298;
points[27].y = 283.9627872644693;
points[27].color = blu;

points[28].x = 152.1031171938624;
points[28].y = 235.26792128487082;
points[28].color = giallo;

points[29].x = 124.15146407631663;
points[29].y = 274.7446530839556;
points[29].color = blu;

points[30].x = 152.36343666297205;
points[30].y = 227.19668556499732;
points[30].color = giallo;

points[31].x = 121.05135040627904;
points[31].y = 265.36512852453205;
points[31].color = blu;

points[32].x = 153.40053746949374;
points[32].y = 217.36551065282057;
points[32].color = giallo;

points[33].x = 118.5700863806236;
points[33].y = 255.7847271099256;
points[33].color = blu;

points[34].x = 155.03511235310782;
points[34].y = 207.6047922383303;
points[34].color = giallo;

points[35].x = 116.03861992784772;
points[35].y = 249.57819489430344;
points[35].color = blu;

points[36].x = 156.51959734826926;
points[36].y = 197.81960809793782;
points[36].color = giallo;

points[37].x = 114.20912696512688;
points[37].y = 239.85160865546285;
points[37].color = blu;

points[38].x = 157.0771580278747;
points[38].y = 188.04255229859805;
points[38].color = giallo;

points[39].x = 112.38947032933534;
points[39].y = 230.1245210558427;
points[39].color = blu;

points[40].x = 155.42129896719769;
points[40].y = 178.2849063153999;
points[40].color = giallo;

points[41].x = 113.22051675820519;
points[41].y = 220.26232625978886;
points[41].color = blu;

points[42].x = 152.13713577322696;
points[42].y = 168.97410381534476;
points[42].color = giallo;

points[43].x = 115.35803956779444;
points[43].y = 210.60061991692922;
points[43].color = blu;

points[44].x = 148.05540747832327;
points[44].y = 159.96554434624855;
points[44].color = giallo;

points[45].x = 117.29064970248997;
points[45].y = 200.89964644867163;
points[45].color = blu;

points[46].x = 143.77113805372065;
points[46].y = 151.0437880684996;
points[46].color = giallo;

points[47].x = 118.15349193383848;
points[47].y = 192.56389469358615;
points[47].color = blu;

points[48].x = 139.46513920839197;
points[48].y = 142.13245268561352;
points[48].color = giallo;

points[49].x = 115.27291266011842;
points[49].y = 183.17572088910046;
points[49].color = blu;

points[50].x = 137.93167507721117;
points[50].y = 138.91762474240025;
points[50].color = giallo;

points[51].x = 111.38254730181065;
points[51].y = 174.07818538917795;
points[51].color = blu;

points[52].x = 138.55439339179665;
points[52].y = 133.75773605333134;
points[52].color = giallo;

points[53].x = 107.175081498462;
points[53].y = 165.11990676681413;
points[53].color = blu;

points[54].x = 138.2602873668891;
points[54].y = 123.87014470813425;
points[54].color = giallo;

points[55].x = 104.15839538482636;
points[55].y = 155.7382791062575;
points[55].color = blu;

points[56].x = 138.21882590224203;
points[56].y = 113.97320926746772;
points[56].color = giallo;

points[57].x = 101.95448394683154;
points[57].y = 146.089636780224;
points[57].color = blu;

points[58].x = 138.94882435453326;
points[58].y = 104.14089330544418;
points[58].color = giallo;

points[59].x = 99.75057250883674;
points[59].y = 136.4409944541905;
points[59].color = blu;

points[60].x = 142.08546687800418;
points[60].y = 95.32701906649409;
points[60].color = giallo;

points[61].x = 98.67012456797721;
points[61].y = 126.62560152702936;
points[61].color = blu;

points[62].x = 144.85716858301228;
points[62].y = 85.86340048747563;
points[62].color = giallo;

points[63].x = 98.65624201813301;
points[63].y = 116.72854910479388;
points[63].color = blu;

points[64].x = 149.42571598553988;
points[64].y = 77.08418160410216;
points[64].color = giallo;

points[65].x = 99.17762831903298;
points[65].y = 106.8845449287471;
points[65].color = blu;

points[66].x = 144.67118243742343;
points[66].y = 77.86969870512311;
points[66].color = giallo;

points[67].x = 101.20251344524821;
points[67].y = 97.20124982591638;
points[67].color = blu;

points[68].x = 154.13708565117201;
points[68].y = 74.98002732065856;
points[68].color = giallo;

points[69].x = 103.60382299724596;
points[69].y = 87.59983134749696;
points[69].color = blu;

points[70].x = 161.40831768641283;
points[70].y = 72.62820608285313;
points[70].color = giallo;

points[71].x = 105.89550071720909;
points[71].y = 77.97225989159178;
points[71].color = blu;

points[72].x = 168.29738444800344;
points[72].y = 72.93466081644384;
points[72].color = giallo;

points[73].x = 109.6384979512498;
points[73].y = 68.90806651469852;
points[73].color = blu;

points[74].x = 176.97883607994132;
points[74].y = 73.92589977020634;
points[74].color = giallo;

points[75].x = 114.38720069821824;
points[75].y = 60.22455647651788;
points[75].color = blu;

points[76].x = 185.379184428053;
points[76].y = 79.15922299666836;
points[76].color = giallo;

points[77].x = 118.9970076922278;
points[77].y = 51.46657635670625;
points[77].color = blu;

points[78].x = 193.14952828427735;
points[78].y = 85.27389649795019;
points[78].color = giallo;

points[79].x = 127.52274227900882;
points[79].y = 46.53421993092368;
points[79].color = blu;

points[80].x = 200.9637061150867;
points[80].y = 91.31549718755241;
points[80].color = giallo;

points[81].x = 136.17084124007548;
points[81].y = 41.721330079509336;
points[81].color = blu;

points[82].x = 209.45509948316942;
points[82].y = 96.39976341098429;
points[82].color = giallo;

points[83].x = 142.7049986236085;
points[83].y = 38.89318302598023;
points[83].color = blu;

points[84].x = 216.91521458999577;
points[84].y = 102.46363520811663;
points[84].color = giallo;

points[85].x = 152.17296986532378;
points[85].y = 36.010294729791504;
points[85].color = blu;

points[86].x = 225.55758470694983;
points[86].y = 107.28680461461988;
points[86].color = giallo;

points[87].x = 161.6713554481429;
points[87].y = 33.333333333333336;
points[87].color = blu;

points[88].x = 234.57327844111214;
points[88].y = 111.34599243807634;
points[88].color = giallo;

points[89].x = 171.54860918606502;
points[89].y = 33.57676158111495;
points[89].color = blu;

points[90].x = 243.9063624400134;
points[90].y = 114.59747715890967;
points[90].color = giallo;

points[91].x = 181.2906307625875;
points[91].y = 35.32219561447736;
points[91].color = blu;

points[92].x = 253.38489852155553;
points[92].y = 117.44543750153252;
points[92].color = giallo;

points[93].x = 190.85794060410572;
points[93].y = 37.60403393237244;
points[93].color = blu;

points[94].x = 263.158547759559;
points[94].y = 118.28626570266272;
points[94].color = giallo;

points[95].x = 199.75917390156147;
points[95].y = 41.93087690924528;
points[95].color = blu;

points[96].x = 273.0556950189184;
points[96].y = 118.28590937467105;
points[96].color = giallo;

points[97].x = 208.23121043089517;
points[97].y = 46.9832158159656;
points[97].color = blu;

points[98].x = 282.9528420250511;
points[98].y = 118.28788115348088;
points[98].color = giallo;

points[99].x = 215.51537744428944;
points[99].y = 53.62926682502941;
points[99].color = blu;

points[100].x = 292.7965335175436;
points[100].y = 117.36229216719981;
points[100].color = giallo;

points[101].x = 223.59377947461894;
points[101].y = 59.34704130454654;
points[101].color = blu;

points[102].x = 302.6279462037866;
points[102].y = 116.22349940509206;
points[102].color = giallo;

points[103].x = 231.2329708891792;
points[103].y = 65.08998833428667;
points[103].color = blu;

points[104].x = 305.0310769705454;
points[104].y = 114.68901477998185;
points[104].color = giallo;

points[105].x = 239.69473290321307;
points[105].y = 70.22341876433079;
points[105].color = blu;

points[106].x = 314.22478554546774;
points[106].y = 118.3536084821062;
points[106].color = giallo;

points[107].x = 248.60356061441925;
points[107].y = 74.51157319081865;
points[107].color = blu;

points[108].x = 322.67194749890336;
points[108].y = 123.50979589802729;
points[108].color = giallo;

points[109].x = 255.8344752344543;
points[109].y = 78.74331830419659;
points[109].color = blu;

points[110].x = 329.4432394254425;
points[110].y = 130.6933474001137;
points[110].color = giallo;

points[111].x = 265.73151025829833;
points[111].y = 78.69780248266422;
points[111].color = blu;

points[112].x = 336.43870165107387;
points[112].y = 137.6612815442526;
points[112].color = giallo;

points[113].x = 275.6286574982177;
points[113].y = 78.69738549921627;
points[113].color = blu;

points[114].x = 342.3717826373977;
points[114].y = 145.5824787077093;
points[114].color = giallo;

points[115].x = 285.2868352320117;
points[115].y = 79.21131304658833;
points[115].color = blu;

points[116].x = 344.7230007369808;
points[116].y = 155.09534934774805;
points[116].color = giallo;

points[117].x = 294.5590320602937;
points[117].y = 79.10530503114809;
points[117].color = blu;

points[118].x = 346.5306488291681;
points[118].y = 164.80557032426228;
points[118].color = giallo;

points[119].x = 304.34712565298383;
points[119].y = 77.64012648128228;
points[119].color = blu;

points[120].x = 348.18097522024823;
points[120].y = 174.56415358030685;
points[120].color = giallo;

points[121].x = 314.02496861042954;
points[121].y = 79.6656965154209;
points[121].color = blu;

points[122].x = 350.24309860740374;
points[122].y = 184.2391612130484;
points[122].color = giallo;

points[123].x = 323.7010946824544;
points[123].y = 81.7456264662916;
points[123].color = blu;

points[124].x = 353.10140138520916;
points[124].y = 193.69295164834048;
points[124].color = giallo;

points[125].x = 333.27811694250227;
points[125].y = 83.7679671595786;
points[125].color = blu;

points[126].x = 357.02444440073486;
points[126].y = 202.77450577702885;
points[126].color = giallo;

points[127].x = 341.5677903377184;
points[127].y = 89.16486649533017;
points[127].color = blu;

points[128].x = 361.11948033393514;
points[128].y = 211.78472017633354;
points[128].color = giallo;

points[129].x = 349.4945462822159;
points[129].y = 95.09108489820933;
points[129].color = blu;

points[130].x = 365.22269107324195;
points[130].y = 220.78983651948872;
points[130].color = giallo;

points[131].x = 356.5308598238018;
points[131].y = 102.0062989876745;
points[131].color = blu;

points[132].x = 370.09014319737133;
points[132].y = 229.40712450805415;
points[132].color = giallo;

points[133].x = 363.3269422344068;
points[133].y = 109.20062111665386;
points[133].color = blu;

points[134].x = 375.63098288281844;
points[134].y = 237.58402797803876;
points[134].color = giallo;

points[135].x = 370.01668511432104;
points[135].y = 116.47980705087217;
points[135].color = blu;

points[136].x = 381.2981217610749;
points[136].y = 245.58837338016494;
points[136].color = giallo;

points[137].x = 375.5035294256666;
points[137].y = 124.66646189477449;
points[137].color = blu;

points[138].x = 386.26686716654166;
points[138].y = 254.14787648828408;
points[138].color = giallo;

points[139].x = 379.6748769032916;
points[139].y = 133.64161563068117;
points[139].color = blu;

points[140].x = 389.9538500607157;
points[140].y = 263.3118548308078;
points[140].color = giallo;

points[141].x = 382.2274696134569;
points[141].y = 143.18235819905337;
points[141].color = blu;

points[142].x = 393.8987798893184;
points[142].y = 272.3676861842207;
points[142].color = giallo;

points[143].x = 383.97283995552215;
points[143].y = 152.9074417908559;
points[143].color = blu;

points[144].x = 395.2127384855251;
points[144].y = 282.17722425538915;
points[144].color = giallo;

points[145].x = 385.16774105591736;
points[145].y = 162.732192969513;
points[145].color = blu;

points[146].x = 395.1994463274223;
points[146].y = 292.0363264557132;
points[146].color = giallo;

points[147].x = 387.39568953881263;
points[147].y = 172.2441699796603;
points[147].color = blu;

points[148].x = 394.41977653894446;
points[148].y = 301.8695166824484;
points[148].color = giallo;

points[149].x = 390.7819778282921;
points[149].y = 181.52740749029988;
points[149].color = blu;

points[150].x = 391.3349428253363;
points[150].y = 311.1559679094654;
points[150].color = giallo;

points[151].x = 394.78941273891013;
points[151].y = 190.56097388063745;
points[151].color = blu;

points[152].x = 389.1208605979163;
points[152].y = 315.77522445502035;
points[152].color = giallo;

points[153].x = 398.27058926617997;
points[153].y = 199.75296659499202;
points[153].color = blu;

points[154].x = 380.85662897368894;
points[154].y = 321.21481911795775;
points[154].color = giallo;

points[155].x = 403.3602449171345;
points[155].y = 208.23119165266303;
points[155].color = blu;

points[156].x = 371.63247810043896;
points[156].y = 324.68440451816645;
points[156].color = giallo;

points[157].x = 408.7789740257039;
points[157].y = 216.5126791739525;
points[157].color = blu;

points[158].x = 362.0035283236987;
points[158].y = 326.4510010154805;
points[158].color = giallo;

points[159].x = 414.10695616224825;
points[159].y = 224.8533104878828;
points[159].color = blu;

points[160].x = 352.1919287493163;
points[160].y = 327.7406572645215;
points[160].color = giallo;

points[161].x = 419.0800783055308;
points[161].y = 233.40296410739612;
points[161].color = blu;

points[162].x = 352.3536224778709;
points[162].y = 333.9939362752329;
points[162].color = giallo;

points[163].x = 423.74136954783324;
points[163].y = 242.13288019587358;
points[163].color = blu;

points[164].x = 343.56480295335126;
points[164].y = 329.4430928668445;
points[164].color = giallo;

points[165].x = 427.8312496060515;
points[165].y = 251.14138750754012;
points[165].color = blu;

points[166].x = 334.7759834288316;
points[166].y = 324.89224945845615;
points[166].color = giallo;

points[167].x = 431.1822425666864;
points[167].y = 260.4539795495027;
points[167].color = blu;

points[168].x = 325.63013084916184;
points[168].y = 321.54770249038796;
points[168].color = giallo;

points[169].x = 433.0998401441061;
points[169].y = 270.05799614695593;
points[169].color = blu;

points[170].x = 315.8273487403196;
points[170].y = 320.2839910028697;
points[170].color = giallo;

points[171].x = 433.9328142043651;
points[171].y = 279.91891717717454;
points[171].color = blu;

points[172].x = 306.16025235285343;
points[172].y = 318.17299566847703;
points[172].color = giallo;

points[173].x = 434.67709131520036;
points[173].y = 289.78161613641333;
points[173].color = blu;

points[174].x = 296.44176455237863;
points[174].y = 316.3009595485606;
points[174].color = giallo;

points[175].x = 434.65990541872674;
points[175].y = 299.67874849923504;
points[175].color = blu;

points[176].x = 288.40549523664527;
points[176].y = 314.92975315504447;
points[176].color = giallo;

points[177].x = 433.0697722557907;
points[177].y = 309.4432525742803;
points[177].color = blu;

points[178].x = 289.531930349307;
points[178].y = 316.03883073717964;
points[178].color = giallo;

points[179].x = 430.52218954854465;
points[179].y = 318.982332730343;
points[179].color = blu;

points[180].x = 279.93380220624994;
points[180].y = 313.6244031628458;
points[180].color = giallo;

points[181].x = 427.0996468688085;
points[181].y = 328.23415977726205;
points[181].color = blu;

points[182].x = 270.33567406319276;
points[182].y = 311.2099755885119;
points[182].color = giallo;

points[183].x = 421.65178591057133;
points[183].y = 336.42315057728433;
points[183].color = blu;

points[184].x = 268.803308184785;
points[184].y = 308.65645512606505;
points[184].color = giallo;

points[185].x = 414.9329546012121;
points[185].y = 343.69025661742336;
points[185].color = blu;

points[186].x = 260.0085299682196;
points[186].y = 304.13989807820246;
points[186].color = giallo;

points[187].x = 407.9758841649093;
points[187].y = 350.6806003610402;
points[187].color = blu;

points[188].x = 251.04154976426776;
points[188].y = 299.9948534865794;
points[188].color = giallo;

points[189].x = 399.5032247217891;
points[189].y = 355.7245015850902;
points[189].color = blu;

points[190].x = 390.2821500286259;
points[190].y = 359.23419653328347;
points[190].color = blu;

points[191].x = 380.6730144986124;
points[191].y = 361.6010145201023;
points[191].color = blu;

points[192].x = 371.1697988448416;
points[192].y = 364.36550662538593;
points[192].color = blu;

points[193].x = 361.5000819174404;
points[193].y = 366.27853712822895;
points[193].color = blu;

points[194].x = 351.640411109898;
points[194].y = 366.6666666666667;
points[194].color = blu;

points[195].x = 341.75835302877886;
points[195].y = 366.1203586549787;
points[195].color = blu;

points[196].x = 332.92345395562;
points[196].y = 362.0072950380525;
points[196].color = blu;

points[197].x = 324.3144411598487;
points[197].y = 357.1248342809654;
points[197].color = blu;

points[198].x = 321.87908460459676;
points[198].y = 357.04393945628595;
points[198].color = blu;

points[199].x = 322.10912258821764;
points[199].y = 360.9266298286049;
points[199].color = blu;

points[200].x = 312.420920331437;
points[200].y = 359.0603542525658;
points[200].color = blu;

points[201].x = 302.77838602343587;
points[201].y = 356.849538995926;
points[201].color = blu;

points[202].x = 293.08601555626825;
points[202].y = 354.84667127951906;
points[202].color = blu;

points[203].x = 287.3662912421476;
points[203].y = 352.33405126591697;
points[203].color = blu;

points[204].x = 277.75636923635307;
points[204].y = 349.9670017512688;
points[204].color = blu;

points[205].x = 268.91899333066885;
points[205].y = 347.9318913768655;
points[205].color = blu;

points[206].x = 268.3916591204781;
points[206].y = 349.4213083280665;
points[206].color = blu;

points[207].x = 258.8028349375541;
points[207].y = 346.97019120495264;
points[207].color = blu;

points[208].x = 249.62842374842793;
points[208].y = 343.65157184952335;
points[208].color = blu;

points[209].x = 243.69187908076867;
points[209].y = 338.81956408173056;
points[209].color = blu;

points[210].x = 237.39581095027788;
points[210].y = 336.85156508477974;
points[210].color = blu;

scaleFactor = 1.5;

/*
points[0].x = 51.43740124556603;
points[0].y = 8.612570356472796;
points[0].color = giallo;

points[1].x = 52.46503777882421;
points[1].y = 11.5;
points[1].color = blu;

points[2].x = 50.43740124556603;
points[2].y = 8.612570356472796;
points[2].color = giallo;

points[3].x = 51.47974257847185;
points[3].y = 11.612570356472796;
points[3].color = blu;

points[4].x = 49.45210604521367;
points[4].y = 8.72514071294559;
points[4].color = giallo;

points[5].x = 50.48449219927502;
points[5].y = 11.638194159031952;
points[5].color = blu;

points[6].x = 48.45210604521367;
points[6].y = 8.72514071294559;
points[6].color = giallo;

points[7].x = 49.49444737811949;
points[7].y = 11.72514071294559;
points[7].color = blu;

points[8].x = 47.45210604521367;
points[8].y = 8.72514071294559;
points[8].color = giallo;

points[9].x = 48.49444737811949;
points[9].y = 11.72514071294559;
points[9].color = blu;

points[10].x = 46.466810844861314;
points[10].y = 8.837711069418386;
points[10].color = giallo;

points[11].x = 47.50221705985074;
points[11].y = 11.769300217379959;
points[11].color = blu;

points[12].x = 45.466810844861314;
points[12].y = 8.837711069418386;
points[12].color = giallo;

points[13].x = 46.509152177767135;
points[13].y = 11.837711069418386;
points[13].color = blu;

points[14].x = 44.47982032835193;
points[14].y = 8.941522776646634;
points[14].color = giallo;

points[15].x = 45.509152177767135;
points[15].y = 11.837711069418386;
points[15].color = blu;

points[16].x = 43.48151564450895;
points[16].y = 8.950281425891182;
points[16].color = giallo;

points[17].x = 44.52385697741478;
points[17].y = 11.950281425891182;
points[17].color = blu;

points[18].x = 42.49657143996792;
points[18].y = 9.072249018078077;
points[18].color = giallo;

points[19].x = 43.53797819766113;
points[19].y = 12.049667521419392;
points[19].color = blu;

points[20].x = 41.510926750951064;
points[20].y = 9.175675110852165;
points[20].color = giallo;

points[21].x = 42.55326656884209;
points[21].y = 12.175417009734637;
points[21].color = blu;

points[22].x = 40.525630043451855;
points[22].y = 9.287992495309568;
points[22].color = giallo;

points[23].x = 41.567758579735646;
points[23].y = 12.281261660534584;
points[23].color = blu;

points[24].x = 39.54033484309949;
points[24].y = 9.400562851782365;
points[24].color = giallo;

points[25].x = 40.578124851271795;
points[25].y = 12.348836931691423;
points[25].color = blu;

points[26].x = 38.555402839862666;
points[26].y = 9.522667947688351;
points[26].color = giallo;

points[27].x = 39.59180110479324;
points[27].y = 12.45391156778312;
points[27].color = blu;

points[28].x = 37.57338387496372;
points[28].y = 9.67025023713118;
points[28].color = giallo;

points[29].x = 38.612090161720424;
points[29].y = 12.625702159929862;
points[29].color = blu;

points[30].x = 36.59122628009496;
points[30].y = 9.805623182455232;
points[30].color = giallo;

points[31].x = 37.62679496352298;
points[31].y = 12.73827392120075;
points[31].color = blu;

points[32].x = 35.61533946782047;
points[32].y = 9.974505121958888;
points[32].color = giallo;

points[33].x = 36.64578328911839;
points[33].y = 12.873792848821406;
points[33].color = blu;

points[34].x = 34.638389409941716;
points[34].y = 10.156074147985452;
points[34].color = giallo;

points[35].x = 35.67193560036834;
points[35].y = 13.075984990619137;
points[35].color = blu;

points[36].x = 33.65910833520551;
points[36].y = 10.305316956057865;
points[36].color = giallo;

points[37].x = 34.69768311478186;
points[37].y = 13.25635698634158;
points[37].color = blu;

points[38].x = 32.69323746987113;
points[38].y = 10.536788569323166;
points[38].color = giallo;

points[39].x = 33.7299101780218;
points[39].y = 13.469404768542384;
points[39].color = blu;

points[40].x = 31.72224081417026;
points[40].y = 10.751584392409738;
points[40].color = giallo;

points[41].x = 32.75099604602515;
points[41].y = 13.644602911762165;
points[41].color = blu;

points[42].x = 30.757669861121183;
points[42].y = 10.989836561048877;
points[42].color = giallo;

points[43].x = 31.788972770687963;
points[43].y = 13.892718344860477;
points[43].color = blu;

points[44].x = 29.78780156587963;
points[44].y = 11.209814351714012;
points[44].color = giallo;

points[45].x = 30.816762938201443;
points[45].y = 14.102899858945104;
points[45].color = blu;

points[46].x = 28.82509711741193;
points[46].y = 11.463725567377953;
points[46].color = giallo;

points[47].x = 29.859925575475618;
points[47].y = 14.381363088617569;
points[47].color = blu;

points[48].x = 27.858593723100608;
points[48].y = 11.705347927051685;
points[48].color = giallo;

points[49].x = 28.89309447756925;
points[49].y = 14.621614248127361;
points[49].color = blu;

points[50].x = 26.918104534723003;
points[50].y = 12.025080902896294;
points[50].color = giallo;

points[51].x = 27.95288702632864;
points[51].y = 14.942525528026719;
points[51].color = blu;

points[52].x = 25.974684184160875;
points[52].y = 12.336553840734245;
points[52].color = giallo;

points[53].x = 27.002182768978045;
points[53].y = 15.22364785910527;
points[53].color = blu;

points[54].x = 25.01789988108644;
points[54].y = 12.613418539555004;
points[54].color = giallo;

points[55].x = 26.04583836714304;
points[55].y = 15.503648631518299;
points[55].color = blu;

points[56].x = 24.077069122239706;
points[56].y = 12.931823623006295;
points[56].color = giallo;

points[57].x = 25.11195008608036;
points[57].y = 15.849680886221405;
points[57].color = blu;

points[58].x = 23.1571822154197;
points[58].y = 13.307714036086905;
points[58].color = giallo;

points[59].x = 24.1839244330994;
points[59].y = 16.193638299294882;
points[59].color = blu;

points[60].x = 22.221036968454932;
points[60].y = 13.650229415335579;
points[60].color = giallo;

points[61].x = 23.25097925681768;
points[61].y = 16.542127730170904;
points[61].color = blu;

points[62].x = 21.29153825718703;
points[62].y = 14.00671726946969;
points[62].color = giallo;

points[63].x = 22.319021264692545;
points[63].y = 16.893773352112902;
points[63].color = blu;

points[64].x = 20.37651252169903;
points[64].y = 14.401115431861925;
points[64].color = giallo;

points[65].x = 21.40777449742499;
points[65].y = 17.295586488737683;
points[65].color = blu;

points[66].x = 19.466007819602595;
points[66].y = 14.80451984031298;
points[66].color = giallo;

points[67].x = 20.49364435286083;
points[67].y = 17.691949483840162;
points[67].color = blu;

points[68].x = 18.572891833014616;
points[68].y = 15.251753722836192;
points[68].color = giallo;

points[69].x = 19.600519414002807;
points[69].y = 18.139161600655513;
points[69].color = blu;

points[70].x = 17.67280308277758;
points[70].y = 15.66135136757055;
points[70].color = giallo;

points[71].x = 18.69498611469181;
points[71].y = 18.5514757077503;
points[71].color = blu;

points[72].x = 16.799249684955434;
points[72].y = 16.137451395631114;
points[72].color = giallo;

points[73].x = 17.822592209148137;
points[73].y = 19.02984979137022;
points[73].color = blu;

points[74].x = 15.905628062235172;
points[74].y = 16.583663070702897;
points[74].color = giallo;

points[75].x = 16.937259763138893;
points[75].y = 19.489161143319194;
points[75].color = blu;

points[76].x = 15.037679547552745;
points[76].y = 17.072238906767126;
points[76].color = giallo;

points[77].x = 16.078985991041172;
points[77].y = 19.993779405208528;
points[77].color = blu;

points[78].x = 14.169621592749175;
points[78].y = 17.561010349176016;
points[78].color = giallo;

points[79].x = 15.234013341071769;
points[79].y = 20.51421280352824;
points[79].color = blu;

points[80].x = 13.337980038280204;
points[80].y = 18.104606631328675;
points[80].color = giallo;

points[81].x = 14.433069544702146;
points[81].y = 21.10288494519896;
points[81].color = blu;

points[82].x = 12.52785806958999;
points[82].y = 18.682863792871792;
points[82].color = giallo;

points[83].x = 13.670789475713924;
points[83].y = 21.74214923218244;
points[83].color = blu;

points[84].x = 11.776205270743118;
points[84].y = 19.33329352845302;
points[84].color = giallo;

points[85].x = 12.903118384875796;
points[85].y = 22.374776253297604;
points[85].color = blu;

points[86].x = 11.007069805930614;
points[86].y = 19.963940920314183;
points[86].color = giallo;

points[87].x = 12.161378858590085;
points[87].y = 23.03686456189263;
points[87].color = blu;

points[88].x = 10.29256895086565;
points[88].y = 20.663033782497784;
points[88].color = giallo;

points[89].x = 11.45427207740354;
points[89].y = 23.74397134307918;
points[89].color = blu;

points[90].x = 9.585462169679099;
points[90].y = 21.370140563684338;
points[90].color = giallo;

points[91].x = 10.747165296216991;
points[91].y = 24.451078124265727;
points[91].color = blu;

points[92].x = 8.878355388492553;
points[92].y = 22.07724734487088;
points[92].color = giallo;

points[93].x = 10.040058515030438;
points[93].y = 25.15818490545228;
points[93].color = blu;

points[94].x = 8.171248607306005;
points[94].y = 22.78435412605743;
points[94].color = giallo;

points[95].x = 9.39539864279921;
points[95].y = 25.915415134156305;
points[95].color = blu;

points[96].x = 7.474216402847138;
points[96].y = 23.50066813640156;
points[96].color = giallo;

points[97].x = 8.7910229917201;
points[97].y = 26.70335800079836;
points[97].color = blu;

points[98].x = 6.825641148091208;
points[98].y = 24.25442986398407;
points[98].color = giallo;

points[99].x = 8.183909034046684;
points[99].y = 27.490746091383706;
points[99].color = blu;

points[100].x = 6.223125614931057;
points[100].y = 25.045709499731856;
points[100].color = giallo;

points[101].x = 7.58660752053054;
points[101].y = 28.28705803878895;
points[101].color = blu;

points[102].x = 5.6157019601365965;
points[102].y = 25.83291106748549;
points[102].color = giallo;

points[103].x = 7.031556462395795;
points[103].y = 29.11361134557084;
points[103].color = blu;

points[104].x = 5.017735157152705;
points[104].y = 26.62686571584579;
points[104].color = giallo;

points[105].x = 6.501217498395362;
points[105].y = 29.956439086673328;
points[105].color = blu;

points[106].x = 4.4610640737643905;
points[106].y = 27.454401339641542;
points[106].color = giallo;

points[107].x = 5.969867883747072;
points[107].y = 30.794574328400223;
points[107].color = blu;

points[108].x = 3.923994163971043;
points[108].y = 28.2939121351494;
points[108].color = giallo;

points[109].x = 5.479063666010711;
points[109].y = 31.661622902265762;
points[109].color = blu;

points[110].x = 3.391424985788929;
points[110].y = 29.13144680269765;
points[110].color = giallo;

points[111].x = 4.9869888659653085;
points[111].y = 32.52783395609149;
points[111].color = blu;

points[112].x = 2.903175945261721;
points[112].y = 29.999753801037073;
points[112].color = giallo;

points[113].x = 4.539779730550769;
points[113].y = 33.42096012988617;
points[113].color = blu;

points[114].x = 2.413651145402168;
points[114].y = 30.867220745143126;
points[114].color = giallo;

points[115].x = 4.143372866490619;
points[115].y = 34.33506985441979;
points[115].color = blu;

points[116].x = 1.9673935624701644;
points[116].y = 31.760815664835423;
points[116].color = giallo;

points[117].x = 3.7415946776337203;
points[117].y = 35.24633328214419;
points[117].color = blu;

points[118].x = 1.524828215784263;
points[118].y = 32.65622258974252;
points[118].color = giallo;

points[119].x = 3.389939150752127;
points[119].y = 36.17828686814319;
points[119].color = blu;

points[120].x = 1.1185982119782427;
points[120].y = 33.565483310772834;
points[120].color = giallo;

points[121].x = 3.041557814486724;
points[121].y = 37.111267888692716;
points[121].color = blu;

points[122].x = 0.7277021033222661;
points[122].y = 34.48201415114645;
points[122].color = giallo;

points[123].x = 2.73193435401142;
points[123].y = 38.05520746284479;
points[123].color = blu;

points[124].x = 0.4154238658020412;
points[124].y = 35.42944606336623;
points[124].color = giallo;

points[125].x = 2.4990133092448867;
points[125].y = 39.02329274759738;
points[125].color = blu;

points[126].x = 0.06493056107712136;
points[126].y = 36.361890573106294;
points[126].color = giallo;

points[127].x = 2.2410223058763528;
points[127].y = 39.97842306552152;
points[127].color = blu;

points[128].x = -0.23694040728493454;
points[128].y = 37.30821607315859;
points[128].color = giallo;

points[129].x = 2.0589128649893658;
points[129].y = 40.95285037091657;
points[129].color = blu;

points[130].x = -0.47286749432623437;
points[130].y = 38.2758866956905;
points[130].color = giallo;

points[131].x = 1.8377110694183865;
points[131].y = 41.9225097057263;
points[131].color = blu;

points[132].x = -0.741798096007141;
points[132].y = 39.22718162144709;
points[132].color = giallo;

points[133].x = 1.725140712945591;
points[133].y = 42.90780490607859;
points[133].color = blu;

points[134].x = -0.9250286582110483;
points[134].y = 40.20374651079193;
points[134].color = giallo;

points[135].x = 1.6125703564727956;
points[135].y = 43.89310010643085;
points[135].color = blu;

points[136].x = -1.1440144708199886;
points[136].y = 41.17374484601457;
points[136].color = giallo;

points[137].x = 1.6125703564727956;
points[137].y = 44.89310010643085;
points[137].color = blu;

points[138].x = -1.2744014632267262;
points[138].y = 42.15809098596309;
points[138].color = giallo;

points[139].x = 1.5142817309088032;
points[139].y = 45.88112065472347;
points[139].color = blu;

points[140].x = -1.3769408565416623;
points[140].y = 43.1437964244839;
points[140].color = giallo;

points[141].x = 1.6125703564727956;
points[141].y = 46.86369050713542;
points[141].color = blu;

points[142].x = -1.3874296435272044;
points[142].y = 44.14338246438772;
points[142].color = giallo;

points[143].x = 1.6298559911543375;
points[143].y = 47.86041679372045;
points[143].color = blu;

points[144].x = -1.4636298494806963;
points[144].y = 45.13135675310205;
points[144].color = giallo;

points[145].x = 1.725140712945591;
points[145].y = 48.848985707487685;
points[145].color = blu;

points[146].x = -1.5;
points[146].y = 46.12867766473999;
points[146].color = giallo;

points[147].x = 1.8739372636666574;
points[147].y = 49.82775815737111;
points[147].color = blu;

points[148].x = -1.3874296435272044;
points[148].y = 47.11397286509227;
points[148].color = giallo;

points[149].x = 2.118378870932202;
points[149].y = 50.79103225996332;
points[149].color = blu;

points[150].x = -1.3874296435272044;
points[150].y = 48.11397286509227;
points[150].color = giallo;

points[151].x = 2.293138089796002;
points[151].y = 51.769022491106526;
points[151].color = blu;

points[152].x = -1.274859287054409;
points[152].y = 49.09926806544454;
points[152].color = giallo;

points[153].x = 2.6013115238494526;
points[153].y = 52.71339580674342;
points[153].color = blu;

points[154].x = -1.1622889305816135;
points[154].y = 50.084563265796824;
points[154].color = giallo;

points[155].x = 2.919458913246651;
points[155].y = 53.654573424626314;
points[155].color = blu;

points[156].x = -0.9535132441353972;
points[156].y = 51.05385669919621;
points[156].color = giallo;

points[157].x = 3.3023210774501863;
points[157].y = 54.57352061782526;
points[157].color = blu;

points[158].x = -0.7998981334497824;
points[158].y = 52.03454462698267;
points[158].color = giallo;

points[159].x = 3.705234457639391;
points[159].y = 55.48227293873133;
points[159].color = blu;

points[160].x = -0.5003979660386821;
points[160].y = 52.98201779570565;
points[160].color = giallo;

points[161].x = 4.1815413003715305;
points[161].y = 56.344983878816606;
points[161].color = blu;

points[162].x = -0.23827473348231767;
points[162].y = 53.94148294121912;
points[162].color = giallo;

points[163].x = 4.678286546588578;
points[163].y = 57.208693333698776;
points[163].color = blu;

points[164].x = 0.10490762809413182;
points[164].y = 54.87610553747628;
points[164].color = giallo;

points[165].x = 5.210778213177233;
points[165].y = 58.05054516904917;
points[165].color = blu;

points[166].x = 0.47750711955232156;
points[166].y = 55.79868588060205;
points[166].color = giallo;

points[167].x = 5.750474851079514;
points[167].y = 58.888075636580105;
points[167].color = blu;

points[168].x = 0.8628042337427383;
points[168].y = 56.71720217272013;
points[168].color = giallo;

points[169].x = 6.266391230820738;
points[169].y = 59.7368055149909;
points[169].color = blu;

points[170].x = 1.3050769924718033;
points[170].y = 57.60465020858379;
points[170].color = giallo;

points[171].x = 6.833683919545995;
points[171].y = 60.55158542217979;
points[171].color = blu;

points[172].x = 1.8586960737251836;
points[172].y = 58.42985025969353;
points[172].color = giallo;

points[173].x = 7.428008721376435;
points[173].y = 61.34796417519015;
points[173].color = blu;

points[174].x = 2.381839566650824;
points[174].y = 59.277369108856945;
points[174].color = giallo;

points[175].x = 8.130723223301535;
points[175].y = 62.0592080043756;
points[175].color = blu;

points[176].x = 2.9073662014428923;
points[176].y = 60.123184736091034;
points[176].color = giallo;

points[177].x = 8.78275583639478;
points[177].y = 62.808165195709016;
points[177].color = blu;

points[178].x = 3.43535090381705;
points[178].y = 60.9674030680278;
points[178].color = giallo;

points[179].x = 9.53700560349822;
points[179].y = 63.456961057311936;
points[179].color = blu;

points[180].x = 3.9641376762688676;
points[180].y = 61.80689903687002;
points[180].color = giallo;

points[181].x = 10.294235832202231;
points[181].y = 64.10162092954315;
points[181].color = blu;

points[182].x = 4.556766009344611;
points[182].y = 62.606666330376534;
points[182].color = giallo;

points[183].x = 11.083461527510808;
points[183].y = 64.7044355412926;
points[183].color = blu;

points[184].x = 5.201362922429404;
points[184].y = 63.3639590633624;
points[184].color = giallo;

points[185].x = 11.871687992499975;
points[185].y = 65.31097239829589;
points[185].color = blu;

points[186].x = 5.848571627625785;
points[186].y = 64.118754344713;
points[186].color = giallo;

points[187].x = 12.710321318669235;
points[187].y = 65.84135392499667;
points[187].color = blu;

points[188].x = 6.5531295758471755;
points[188].y = 64.82829607325297;
points[188].color = giallo;

points[189].x = 13.547505474135654;
points[189].y = 66.37466369777985;
points[189].color = blu;

points[190].x = 7.305453201923078;
points[190].y = 65.4797401654032;
points[190].color = giallo;

points[191].x = 14.415779126811078;
points[191].y = 66.86298324178978;
points[191].color = blu;

points[192].x = 8.032638135936365;
points[192].y = 66.16301477279396;
points[192].color = giallo;

points[193].x = 15.281655422802142;
points[193].y = 67.35599721021629;
points[193].color = blu;

points[194].x = 8.805288510971765;
points[194].y = 66.78851750580186;
points[194].color = giallo;

points[195].x = 16.17527114691749;
points[195].y = 67.80221900613411;
points[195].color = blu;

points[196].x = 9.583780468743024;
points[196].y = 67.40513766102458;
points[196].color = giallo;

points[197].x = 17.06886563103243;
points[197].y = 68.24847734708423;
points[197].color = blu;

points[198].x = 10.395160158392425;
points[198].y = 67.98103063673314;
points[198].color = giallo;

points[199].x = 17.9820783375061;
points[199].y = 68.64635715922472;
points[199].color = blu;

points[200].x = 11.233116664207225;
points[200].y = 68.51273703964912;
points[200].color = giallo;

points[201].x = 18.892692037573564;
points[201].y = 69.04975542831667;
points[201].color = blu;

points[202].x = 12.072598029480286;
points[202].y = 69.04154996387751;
points[202].color = giallo;

points[203].x = 19.809430216008526;
points[203].y = 69.43840065236199;
points[203].color = blu;

points[204].x = 12.938462620131979;
points[204].y = 69.5345853548257;
points[204].color = giallo;

points[205].x = 20.731436437511373;
points[205].y = 69.81340936668641;
points[205].color = blu;

points[206].x = 13.80672687884991;
points[206].y = 70.02292476068999;
points[206].color = giallo;

points[207].x = 21.6645015324917;
points[207].y = 70.16223014312706;
points[207].color = blu;

points[208].x = 14.699928554897634;
points[208].y = 70.46995713340704;
points[208].color = giallo;

points[209].x = 22.58130706385924;
points[209].y = 70.55245749116233;
points[209].color = blu;

points[210].x = 15.593147387334865;
points[210].y = 70.91695087531895;
points[210].color = giallo;

points[211].x = 23.52983044442004;
points[211].y = 70.8620782937987;
points[211].color = blu;

points[212].x = 16.498105969737242;
points[212].y = 71.33652031871728;
points[212].color = giallo;

points[213].x = 24.462055168466502;
points[213].y = 71.21310674034628;
points[213].color = blu;

points[214].x = 17.403412800897378;
points[214].y = 71.75358268666906;
points[214].color = giallo;

points[215].x = 25.382513052036344;
points[215].y = 71.59150011694666;
points[215].color = blu;

points[216].x = 18.324699985914517;
points[216].y = 72.13108488615565;
points[216].color = giallo;

points[217].x = 26.30542982347657;
points[217].y = 71.96664312843188;
points[217].color = blu;

points[218].x = 19.243719864612554;
points[218].y = 72.51372474065224;
points[218].color = giallo;

points[219].x = 27.231769625562592;
points[219].y = 72.31963093155666;
points[219].color = blu;

points[220].x = 20.17468945429401;
points[220].y = 72.8671951427394;
points[220].color = giallo;

points[221].x = 28.163832713329484;
points[221].y = 72.67093240237686;
points[221].color = blu;

points[222].x = 21.10502065283781;
points[222].y = 73.22204306332367;
points[222].color = giallo;

points[223].x = 29.095878666539388;
points[223].y = 73.02237812585204;
points[223].color = blu;

points[224].x = 22.030012957461654;
points[224].y = 73.59210290912421;
points[224].color = giallo;

points[225].x = 30.00445289875811;
points[225].y = 73.43026295190973;
points[225].color = blu;

points[226].x = 22.96394879577391;
points[226].y = 73.93794234061804;
points[226].color = giallo;

points[227].x = 30.908404409043936;
points[227].y = 73.8456379588252;
points[227].color = blu;

points[228].x = 23.881890312731162;
points[228].y = 74.32396941440756;
points[228].color = giallo;

points[229].x = 31.794140623794284;
points[229].y = 74.29188106789182;
points[229].color = blu;

points[230].x = 24.830022903836138;
points[230].y = 74.63450712367569;
points[230].color = giallo;

points[231].x = 32.67983779865927;
points[231].y = 74.73827031306918;
points[231].color = blu;

points[232].x = 25.758071138813424;
points[232].y = 74.98415068281294;
points[232].color = giallo;

points[233].x = 33.56049223165637;
points[233].y = 75.19717842859981;
points[233].color = blu;

points[234].x = 26.676588982798872;
points[234].y = 75.36996604256862;
points[234].color = giallo;

points[235].x = 34.42327777771635;
points[235].y = 75.6923507329296;
points[235].color = blu;

points[236].x = 27.599003119967982;
points[236].y = 75.7434779909684;
points[236].color = giallo;

points[237].x = 35.26743542255504;
points[237].y = 76.2200001541109;
points[237].color = blu;

points[238].x = 28.513826064125766;
points[238].y = 76.136971286495;
points[238].color = giallo;

points[239].x = 36.10074480086309;
points[239].y = 76.76789804116862;
points[239].color = blu;

points[240].x = 29.402302715711066;
points[240].y = 76.57711461487173;
points[240].color = giallo;

points[241].x = 36.87236099601437;
points[241].y = 77.39644402956542;
points[241].color = blu;

points[242].x = 30.3052193773138;
points[242].y = 76.97957838794686;
points[242].color = giallo;

points[243].x = 37.5784541989946;
points[243].y = 78.10453555558458;
points[243].color = blu;

points[244].x = 31.19197433854464;
points[244].y = 77.42311589402598;
points[244].color = giallo;

points[245].x = 38.15945671416327;
points[245].y = 78.91249377720949;
points[245].color = blu;

points[246].x = 32.078802441074195;
points[246].y = 77.86650570471852;
points[246].color = giallo;

points[247].x = 38.66647646244357;
points[247].y = 79.76882566295858;
points[247].color = blu;

points[248].x = 32.920962462625326;
points[248].y = 78.3980602570221;
points[248].color = giallo;

points[249].x = 39.09029321769275;
points[249].y = 80.66734869806919;
points[249].color = blu;

points[250].x = 33.78595375361393;
points[250].y = 78.88892163748291;
points[250].color = giallo;

points[251].x = 39.42083995435187;
points[251].y = 81.60574933873744;
points[251].color = blu;

points[252].x = 34.595089089582636;
points[252].y = 79.4682620152806;
points[252].color = giallo;

points[253].x = 39.63913893288039;
points[253].y = 82.575851202531;
points[253].color = blu;

points[254].x = 35.2945703272266;
points[254].y = 80.15908480122107;
points[254].color = giallo;

points[255].x = 39.85618457007659;
points[255].y = 83.54617110554025;
points[255].color = blu;

points[256].x = 35.84447391950561;
points[256].y = 80.98568030113533;
points[256].color = giallo;

points[257].x = 39.96293530657725;
points[257].y = 84.53208184678726;
points[257].color = blu;

points[258].x = 36.266561531043905;
points[258].y = 81.88637418858131;
points[258].color = giallo;

points[259].x = 39.99906191369606;
points[259].y = 85.52942938875827;
points[259].color = blu;

points[260].x = 36.553092018918;
points[260].y = 82.83401400115989;
points[260].color = giallo;

points[261].x = 39.99906191369606;
points[261].y = 86.52942938875827;
points[261].color = blu;

points[262].x = 36.756145857906944;
points[262].y = 83.80807165680153;
points[262].color = giallo;

points[263].x = 40.110918579630166;
points[263].y = 87.51473186770144;
points[263].color = blu;

points[264].x = 36.88709367982749;
points[264].y = 84.78897234485882;
points[264].color = giallo;

points[265].x = 40.111632270168855;
points[265].y = 88.5147245891105;
points[265].color = blu;

points[266].x = 36.99906191369606;
points[266].y = 85.77427325942922;
points[266].color = giallo;

points[267].x = 40.111632270168855;
points[267].y = 89.5147245891105;
points[267].color = blu;

points[268].x = 36.99906191369606;
points[268].y = 86.77427325942922;
points[268].color = giallo;

points[269].x = 40.111632270168855;
points[269].y = 90.5147245891105;
points[269].color = blu;

points[270].x = 37.034326328277864;
points[270].y = 87.77171507812623;
points[270].color = giallo;

points[271].x = 40.21990520579402;
points[271].y = 91.50012833141298;
points[271].color = blu;

points[272].x = 37.111632270168855;
points[272].y = 88.75956845978143;
points[272].color = giallo;

points[273].x = 40.31532951838549;
points[273].y = 92.48652639409458;
points[273].color = blu;

points[274].x = 37.111632270168855;
points[274].y = 89.75956845978143;
points[274].color = giallo;

points[275].x = 40.51602732789427;
points[275].y = 93.45519444443757;
points[275].color = blu;

points[276].x = 37.111632270168855;
points[276].y = 90.75956845978143;
points[276].color = giallo;

points[277].x = 40.8935399130572;
points[277].y = 94.3727302323589;
points[277].color = blu;

points[278].x = 37.13387622065418;
points[278].y = 91.75828886286916;
points[278].color = giallo;

points[279].x = 41.33359045533622;
points[279].y = 95.26104612547586;
points[279].color = blu;

points[280].x = 37.22886273548707;
points[280].y = 92.74474120245031;
points[280].color = giallo;

points[281].x = 41.85009341432321;
points[281].y = 96.10925082420373;
points[281].color = blu;

points[282].x = 37.397454884609765;
points[282].y = 93.72436965859218;
points[282].color = giallo;

points[283].x = 42.41796239276365;
points[283].y = 96.92830637596407;
points[283].color = blu;

points[284].x = 37.6823853331597;
points[284].y = 94.67902667850066;
points[284].color = giallo;

points[285].x = 43.01468472331617;
points[285].y = 97.72263267243946;
points[285].color = blu;

points[286].x = 38.07973491125753;
points[286].y = 95.59070052979486;
points[286].color = giallo;

points[287].x = 43.715895418975414;
points[287].y = 98.4351402565961;
points[287].color = blu;

points[288].x = 38.52536742187391;
points[288].y = 96.48258393881522;
points[288].color = giallo;

points[289].x = 44.42799425039638;
points[289].y = 99.13594219951968;
points[289].color = blu;

points[290].x = 39.01962033823431;
points[290].y = 97.34776563958042;
points[290].color = giallo;

points[291].x = 45.199041391210756;
points[291].y = 99.7611400476812;
points[291].color = blu;

points[292].x = 39.57393416545734;
points[292].y = 98.17248931121541;
points[292].color = giallo;

points[293].x = 46.02748430107673;
points[293].y = 100.309894061169;
points[293].color = blu;

points[294].x = 40.148119034866994;
points[294].y = 98.98613731463676;
points[294].color = giallo;

points[295].x = 46.878517367065626;
points[295].y = 100.82570186773546;
points[295].color = blu;

points[296].x = 40.79277890709824;
points[296].y = 99.7433675433408;
points[296].color = giallo;

points[297].x = 47.7852939875576;
points[297].y = 101.2332352155009;
points[297].color = blu;

points[298].x = 41.43744809963899;
points[298].y = 100.4977522498134;
points[298].color = giallo;

points[299].x = 48.736167568747916;
points[299].y = 101.52781460811919;
points[299].color = blu;

points[300].x = 42.14520210446922;
points[300].y = 101.20107514497289;
points[300].color = giallo;

points[301].x = 49.704277491288714;
points[301].y = 101.70825515947467;
points[301].color = blu;

points[302].x = 42.89889012240352;
points[302].y = 101.8494787586461;
points[302].color = giallo;

points[303].x = 50.68957269164094;
points[303].y = 101.82082551594746;
points[303].color = blu;

points[304].x = 43.705949122110525;
points[304].y = 102.43139704540751;
points[304].color = giallo;

points[305].x = 51.68957269164094;
points[305].y = 101.82082551594746;
points[305].color = blu;

points[306].x = 44.52824215642567;
points[306].y = 102.98918214719598;
points[306].color = giallo;

points[307].x = 52.68957269164094;
points[307].y = 101.82082551594746;
points[307].color = blu;

points[308].x = 45.39136467924035;
points[308].y = 103.4868706243076;
points[308].color = giallo;

points[309].x = 53.68957269164094;
points[309].y = 101.82082551594746;
points[309].color = blu;

points[310].x = 46.28346398165121;
points[310].y = 103.93212074687477;
points[310].color = giallo;

points[311].x = 54.68957269164094;
points[311].y = 101.82082551594746;
points[311].color = blu;

points[312].x = 47.217047958341375;
points[312].y = 104.28016930015269;
points[312].color = giallo;

points[313].x = 55.68957269164094;
points[313].y = 101.82082551594746;
points[313].color = blu;

points[314].x = 48.16493350597927;
points[314].y = 104.57630348774832;
points[314].color = giallo;

points[315].x = 56.68957269164094;
points[315].y = 101.82082551594746;
points[315].color = blu;

points[316].x = 49.14918373615674;
points[316].y = 104.70825515947467;
points[316].color = giallo;

points[317].x = 57.68957269164094;
points[317].y = 101.82082551594746;
points[317].color = blu;

points[318].x = 50.134478936508955;
points[318].y = 104.82082551594746;
points[318].color = giallo;

points[319].x = 58.68957269164094;
points[319].y = 101.82082551594746;
points[319].color = blu;

points[320].x = 51.134478936508955;
points[320].y = 104.82082551594746;
points[320].color = giallo;

points[321].x = 59.68957269164094;
points[321].y = 101.82082551594746;
points[321].color = blu;

points[322].x = 52.134478936508955;
points[322].y = 104.82082551594746;
points[322].color = giallo;

points[323].x = 60.68957269164094;
points[323].y = 101.82082551594746;
points[323].color = blu;

points[324].x = 53.134478936508955;
points[324].y = 104.82082551594746;
points[324].color = giallo;

points[325].x = 61.68957269164094;
points[325].y = 101.82082551594746;
points[325].color = blu;

points[326].x = 54.134478936508955;
points[326].y = 104.82082551594746;
points[326].color = giallo;

points[327].x = 62.68957269164094;
points[327].y = 101.82082551594746;
points[327].color = blu;

points[328].x = 55.134478936508955;
points[328].y = 104.82082551594746;
points[328].color = giallo;

points[329].x = 63.67486789199316;
points[329].y = 101.70825515947467;
points[329].color = blu;

points[330].x = 56.134478936508955;
points[330].y = 104.82082551594746;
points[330].color = giallo;

points[331].x = 64.67486789199316;
points[331].y = 101.70825515947467;
points[331].color = blu;

points[332].x = 57.134478936508955;
points[332].y = 104.82082551594746;
points[332].color = giallo;

points[333].x = 65.67486789199316;
points[333].y = 101.70825515947467;
points[333].color = blu;

points[334].x = 58.134478936508955;
points[334].y = 104.82082551594746;
points[334].color = giallo;

points[335].x = 66.67486789199316;
points[335].y = 101.70825515947467;
points[335].color = blu;

points[336].x = 59.134478936508955;
points[336].y = 104.82082551594746;
points[336].color = giallo;

points[337].x = 67.67486789199316;
points[337].y = 101.70825515947467;
points[337].color = blu;

points[338].x = 60.134478936508955;
points[338].y = 104.82082551594746;
points[338].color = giallo;

points[339].x = 68.67486789199316;
points[339].y = 101.70825515947467;
points[339].color = blu;

points[340].x = 61.134478936508955;
points[340].y = 104.82082551594746;
points[340].color = giallo;

points[341].x = 69.67486789199316;
points[341].y = 101.70825515947467;
points[341].color = blu;

points[342].x = 62.134478936508955;
points[342].y = 104.82082551594746;
points[342].color = giallo;

points[343].x = 70.67486789199316;
points[343].y = 101.70825515947467;
points[343].color = blu;

points[344].x = 63.134478936508955;
points[344].y = 104.82082551594746;
points[344].color = giallo;

points[345].x = 71.67486789199316;
points[345].y = 101.70825515947467;
points[345].color = blu;

points[346].x = 64.1197741368612;
points[346].y = 104.70825515947467;
points[346].color = giallo;

points[347].x = 72.67486789199316;
points[347].y = 101.70825515947467;
points[347].color = blu;

points[348].x = 65.1197741368612;
points[348].y = 104.70825515947467;
points[348].color = giallo;

points[349].x = 73.67486789199316;
points[349].y = 101.70825515947467;
points[349].color = blu;

points[350].x = 66.1197741368612;
points[350].y = 104.70825515947467;
points[350].color = giallo;

points[351].x = 74.67486789199316;
points[351].y = 101.70825515947467;
points[351].color = blu;

points[352].x = 67.1197741368612;
points[352].y = 104.70825515947467;
points[352].color = giallo;

points[353].x = 75.67486789199316;
points[353].y = 101.70825515947467;
points[353].color = blu;

points[354].x = 68.1197741368612;
points[354].y = 104.70825515947467;
points[354].color = giallo;

points[355].x = 76.67486789199316;
points[355].y = 101.70825515947467;
points[355].color = blu;

points[356].x = 69.1197741368612;
points[356].y = 104.70825515947467;
points[356].color = giallo;

points[357].x = 77.67486789199316;
points[357].y = 101.70825515947467;
points[357].color = blu;

points[358].x = 70.1197741368612;
points[358].y = 104.70825515947467;
points[358].color = giallo;

points[359].x = 78.67486789199316;
points[359].y = 101.70825515947467;
points[359].color = blu;

points[360].x = 71.1197741368612;
points[360].y = 104.70825515947467;
points[360].color = giallo;

points[361].x = 79.67486789199316;
points[361].y = 101.70825515947467;
points[361].color = blu;

points[362].x = 72.1197741368612;
points[362].y = 104.70825515947467;
points[362].color = giallo;

points[363].x = 80.67486789199316;
points[363].y = 101.70825515947467;
points[363].color = blu;

points[364].x = 73.1197741368612;
points[364].y = 104.70825515947467;
points[364].color = giallo;

points[365].x = 81.67486789199316;
points[365].y = 101.70825515947467;
points[365].color = blu;

points[366].x = 74.1197741368612;
points[366].y = 104.70825515947467;
points[366].color = giallo;

points[367].x = 82.67486789199316;
points[367].y = 101.70825515947467;
points[367].color = blu;

points[368].x = 75.1197741368612;
points[368].y = 104.70825515947467;
points[368].color = giallo;

points[369].x = 83.67486789199316;
points[369].y = 101.70825515947467;
points[369].color = blu;

points[370].x = 76.1197741368612;
points[370].y = 104.70825515947467;
points[370].color = giallo;

points[371].x = 84.67486789199316;
points[371].y = 101.70825515947467;
points[371].color = blu;

points[372].x = 77.1197741368612;
points[372].y = 104.70825515947467;
points[372].color = giallo;

points[373].x = 85.67486789199316;
points[373].y = 101.70825515947467;
points[373].color = blu;

points[374].x = 78.1197741368612;
points[374].y = 104.70825515947467;
points[374].color = giallo;

points[375].x = 86.67486789199316;
points[375].y = 101.70825515947467;
points[375].color = blu;

points[376].x = 79.1197741368612;
points[376].y = 104.70825515947467;
points[376].color = giallo;

points[377].x = 87.67486789199316;
points[377].y = 101.70825515947467;
points[377].color = blu;

points[378].x = 80.1197741368612;
points[378].y = 104.70825515947467;
points[378].color = giallo;

points[379].x = 88.67486789199316;
points[379].y = 101.70825515947467;
points[379].color = blu;

points[380].x = 81.1197741368612;
points[380].y = 104.70825515947467;
points[380].color = giallo;

points[381].x = 89.67486789199316;
points[381].y = 101.70825515947467;
points[381].color = blu;

points[382].x = 82.1197741368612;
points[382].y = 104.70825515947467;
points[382].color = giallo;

points[383].x = 90.67486789199316;
points[383].y = 101.70825515947467;
points[383].color = blu;

points[384].x = 83.1197741368612;
points[384].y = 104.70825515947467;
points[384].color = giallo;

points[385].x = 91.67486789199316;
points[385].y = 101.70825515947467;
points[385].color = blu;

points[386].x = 84.1197741368612;
points[386].y = 104.70825515947467;
points[386].color = giallo;

points[387].x = 92.67486789199316;
points[387].y = 101.70825515947467;
points[387].color = blu;

points[388].x = 85.1197741368612;
points[388].y = 104.70825515947467;
points[388].color = giallo;

points[389].x = 93.67486789199316;
points[389].y = 101.70825515947467;
points[389].color = blu;

points[390].x = 86.1197741368612;
points[390].y = 104.70825515947467;
points[390].color = giallo;

points[391].x = 94.67486789199316;
points[391].y = 101.70825515947467;
points[391].color = blu;

points[392].x = 87.1197741368612;
points[392].y = 104.70825515947467;
points[392].color = giallo;

points[393].x = 95.67486789199316;
points[393].y = 101.70825515947467;
points[393].color = blu;

points[394].x = 88.1197741368612;
points[394].y = 104.70825515947467;
points[394].color = giallo;

points[395].x = 96.67486789199316;
points[395].y = 101.70825515947467;
points[395].color = blu;

points[396].x = 89.1197741368612;
points[396].y = 104.70825515947467;
points[396].color = giallo;

points[397].x = 97.67486789199316;
points[397].y = 101.70825515947467;
points[397].color = blu;

points[398].x = 90.1197741368612;
points[398].y = 104.70825515947467;
points[398].color = giallo;

points[399].x = 98.67486789199316;
points[399].y = 101.70825515947467;
points[399].color = blu;

points[400].x = 91.1197741368612;
points[400].y = 104.70825515947467;
points[400].color = giallo;

points[401].x = 99.67486789199316;
points[401].y = 101.70825515947467;
points[401].color = blu;

points[402].x = 92.1197741368612;
points[402].y = 104.70825515947467;
points[402].color = giallo;

points[403].x = 100.67486789199316;
points[403].y = 101.70825515947467;
points[403].color = blu;

points[404].x = 93.1197741368612;
points[404].y = 104.70825515947467;
points[404].color = giallo;

points[405].x = 101.67486789199316;
points[405].y = 101.70825515947467;
points[405].color = blu;

points[406].x = 94.1197741368612;
points[406].y = 104.70825515947467;
points[406].color = giallo;

points[407].x = 102.67486789199316;
points[407].y = 101.70825515947467;
points[407].color = blu;

points[408].x = 95.1197741368612;
points[408].y = 104.70825515947467;
points[408].color = giallo;

points[409].x = 103.67486789199316;
points[409].y = 101.70825515947467;
points[409].color = blu;

points[410].x = 96.1197741368612;
points[410].y = 104.70825515947467;
points[410].color = giallo;

points[411].x = 104.67486789199316;
points[411].y = 101.70825515947467;
points[411].color = blu;

points[412].x = 97.1197741368612;
points[412].y = 104.70825515947467;
points[412].color = giallo;

points[413].x = 105.67486789199316;
points[413].y = 101.70825515947467;
points[413].color = blu;

points[414].x = 98.1197741368612;
points[414].y = 104.70825515947467;
points[414].color = giallo;

points[415].x = 106.67486789199316;
points[415].y = 101.70825515947467;
points[415].color = blu;

points[416].x = 99.1197741368612;
points[416].y = 104.70825515947467;
points[416].color = giallo;

points[417].x = 107.67486789199316;
points[417].y = 101.70825515947467;
points[417].color = blu;

points[418].x = 100.1197741368612;
points[418].y = 104.70825515947467;
points[418].color = giallo;

points[419].x = 108.67486789199316;
points[419].y = 101.70825515947467;
points[419].color = blu;

points[420].x = 101.1197741368612;
points[420].y = 104.70825515947467;
points[420].color = giallo;

points[421].x = 109.67486789199316;
points[421].y = 101.70825515947467;
points[421].color = blu;

points[422].x = 102.1197741368612;
points[422].y = 104.70825515947467;
points[422].color = giallo;

points[423].x = 110.67486789199316;
points[423].y = 101.70825515947467;
points[423].color = blu;

points[424].x = 103.1197741368612;
points[424].y = 104.70825515947467;
points[424].color = giallo;

points[425].x = 111.67486789199316;
points[425].y = 101.70825515947467;
points[425].color = blu;

points[426].x = 104.1197741368612;
points[426].y = 104.70825515947467;
points[426].color = giallo;

points[427].x = 112.67486789199316;
points[427].y = 101.70825515947467;
points[427].color = blu;

points[428].x = 105.1197741368612;
points[428].y = 104.70825515947467;
points[428].color = giallo;

points[429].x = 113.67486789199316;
points[429].y = 101.70825515947467;
points[429].color = blu;

points[430].x = 106.1197741368612;
points[430].y = 104.70825515947467;
points[430].color = giallo;

points[431].x = 114.67486789199316;
points[431].y = 101.70825515947467;
points[431].color = blu;

points[432].x = 107.1197741368612;
points[432].y = 104.70825515947467;
points[432].color = giallo;

points[433].x = 115.67486789199316;
points[433].y = 101.70825515947467;
points[433].color = blu;

points[434].x = 108.1197741368612;
points[434].y = 104.70825515947467;
points[434].color = giallo;

points[435].x = 116.67486789199316;
points[435].y = 101.70825515947467;
points[435].color = blu;

points[436].x = 109.1197741368612;
points[436].y = 104.70825515947467;
points[436].color = giallo;

points[437].x = 117.67486789199316;
points[437].y = 101.70825515947467;
points[437].color = blu;

points[438].x = 110.1197741368612;
points[438].y = 104.70825515947467;
points[438].color = giallo;

points[439].x = 118.67486789199316;
points[439].y = 101.70825515947467;
points[439].color = blu;

points[440].x = 111.1197741368612;
points[440].y = 104.70825515947467;
points[440].color = giallo;

points[441].x = 119.67486789199316;
points[441].y = 101.70825515947467;
points[441].color = blu;

points[442].x = 112.1197741368612;
points[442].y = 104.70825515947467;
points[442].color = giallo;

points[443].x = 120.66760233161531;
points[443].y = 101.6673561862359;
points[443].color = blu;

points[444].x = 113.1197741368612;
points[444].y = 104.70825515947467;
points[444].color = giallo;

points[445].x = 121.66016309234547;
points[445].y = 101.59568480300187;
points[445].color = blu;

points[446].x = 114.1197741368612;
points[446].y = 104.70825515947467;
points[446].color = giallo;

points[447].x = 122.66016309234547;
points[447].y = 101.59568480300187;
points[447].color = blu;

points[448].x = 115.1197741368612;
points[448].y = 104.70825515947467;
points[448].color = giallo;

points[449].x = 123.66016309234547;
points[449].y = 101.59568480300187;
points[449].color = blu;

points[450].x = 116.1197741368612;
points[450].y = 104.70825515947467;
points[450].color = giallo;

points[451].x = 124.66016309234547;
points[451].y = 101.59568480300187;
points[451].color = blu;

points[452].x = 117.1197741368612;
points[452].y = 104.70825515947467;
points[452].color = giallo;

points[453].x = 125.66016309234547;
points[453].y = 101.59568480300187;
points[453].color = blu;

points[454].x = 118.1197741368612;
points[454].y = 104.70825515947467;
points[454].color = giallo;

points[455].x = 126.66016309234547;
points[455].y = 101.59568480300187;
points[455].color = blu;

points[456].x = 119.1197741368612;
points[456].y = 104.70825515947467;
points[456].color = giallo;

points[457].x = 127.66016309234547;
points[457].y = 101.59568480300187;
points[457].color = blu;

points[458].x = 120.1197741368612;
points[458].y = 104.70825515947467;
points[458].color = giallo;

points[459].x = 128.66016309234547;
points[459].y = 101.59568480300187;
points[459].color = blu;

points[460].x = 121.11968400898456;
points[460].y = 104.70445915069449;
points[460].color = giallo;

points[461].x = 129.66016309234547;
points[461].y = 101.59568480300187;
points[461].color = blu;

points[462].x = 122.10506933721346;
points[462].y = 104.59568480300187;
points[462].color = giallo;

points[463].x = 130.66016309234547;
points[463].y = 101.59568480300187;
points[463].color = blu;

points[464].x = 123.10506933721346;
points[464].y = 104.59568480300187;
points[464].color = giallo;

points[465].x = 131.66016309234547;
points[465].y = 101.59568480300187;
points[465].color = blu;

points[466].x = 124.10506933721346;
points[466].y = 104.59568480300187;
points[466].color = giallo;

points[467].x = 132.66016309234547;
points[467].y = 101.59568480300187;
points[467].color = blu;

points[468].x = 125.10506933721346;
points[468].y = 104.59568480300187;
points[468].color = giallo;

points[469].x = 133.66016309234547;
points[469].y = 101.59568480300187;
points[469].color = blu;

points[470].x = 126.10506933721346;
points[470].y = 104.59568480300187;
points[470].color = giallo;

points[471].x = 134.66016309234547;
points[471].y = 101.59568480300187;
points[471].color = blu;

points[472].x = 127.10506933721346;
points[472].y = 104.59568480300187;
points[472].color = giallo;

points[473].x = 135.66001653589888;
points[473].y = 101.60093454425183;
points[473].color = blu;

points[474].x = 128.10506933721348;
points[474].y = 104.59568480300187;
points[474].color = giallo;

points[475].x = 136.6454582926978;
points[475].y = 101.70825515947467;
points[475].color = blu;

points[476].x = 129.10506933721348;
points[476].y = 104.59568480300187;
points[476].color = giallo;

points[477].x = 137.6454582926978;
points[477].y = 101.70825515947467;
points[477].color = blu;

points[478].x = 130.10506933721348;
points[478].y = 104.59568480300187;
points[478].color = giallo;

points[479].x = 138.6454582926978;
points[479].y = 101.70825515947467;
points[479].color = blu;

points[480].x = 131.10506933721348;
points[480].y = 104.59568480300187;
points[480].color = giallo;

points[481].x = 139.6454582926978;
points[481].y = 101.70825515947467;
points[481].color = blu;

points[482].x = 132.10506933721348;
points[482].y = 104.59568480300187;
points[482].color = giallo;

points[483].x = 140.6454582926978;
points[483].y = 101.70825515947467;
points[483].color = blu;

points[484].x = 133.10506933721348;
points[484].y = 104.59568480300187;
points[484].color = giallo;

points[485].x = 141.6454582926978;
points[485].y = 101.70825515947467;
points[485].color = blu;

points[486].x = 134.10506933721348;
points[486].y = 104.59568480300187;
points[486].color = giallo;

points[487].x = 142.6454582926978;
points[487].y = 101.70825515947467;
points[487].color = blu;

points[488].x = 135.10307317210544;
points[488].y = 104.60604564710019;
points[488].color = giallo;

points[489].x = 143.6454582926978;
points[489].y = 101.70825515947467;
points[489].color = blu;

points[490].x = 136.09036453756582;
points[490].y = 104.70825515947467;
points[490].color = giallo;

points[491].x = 144.6454582926978;
points[491].y = 101.70825515947467;
points[491].color = blu;

points[492].x = 137.09036453756582;
points[492].y = 104.70825515947467;
points[492].color = giallo;

points[493].x = 145.6454582926978;
points[493].y = 101.70825515947467;
points[493].color = blu;

points[494].x = 138.09036453756582;
points[494].y = 104.70825515947467;
points[494].color = giallo;

points[495].x = 146.6454582926978;
points[495].y = 101.70825515947467;
points[495].color = blu;

points[496].x = 139.09036453756582;
points[496].y = 104.70825515947467;
points[496].color = giallo;

points[497].x = 147.6454582926978;
points[497].y = 101.70825515947467;
points[497].color = blu;

points[498].x = 140.09036453756582;
points[498].y = 104.70825515947467;
points[498].color = giallo;

points[499].x = 148.6454582926978;
points[499].y = 101.70825515947467;
points[499].color = blu;

points[500].x = 141.09036453756582;
points[500].y = 104.70825515947467;
points[500].color = giallo;

points[501].x = 149.6454582926978;
points[501].y = 101.70825515947467;
points[501].color = blu;

points[502].x = 142.09036453756582;
points[502].y = 104.70825515947467;
points[502].color = giallo;

points[503].x = 150.6454582926978;
points[503].y = 101.70825515947467;
points[503].color = blu;

points[504].x = 143.09036453756582;
points[504].y = 104.70825515947467;
points[504].color = giallo;

points[505].x = 151.6454582926978;
points[505].y = 101.70825515947467;
points[505].color = blu;

points[506].x = 144.09036453756582;
points[506].y = 104.70825515947467;
points[506].color = giallo;

points[507].x = 152.6454582926978;
points[507].y = 101.70825515947467;
points[507].color = blu;

points[508].x = 145.09036453756582;
points[508].y = 104.70825515947467;
points[508].color = giallo;

points[509].x = 153.6454582926978;
points[509].y = 101.70825515947467;
points[509].color = blu;

points[510].x = 146.09036453756582;
points[510].y = 104.70825515947467;
points[510].color = giallo;

points[511].x = 154.6454582926978;
points[511].y = 101.70825515947467;
points[511].color = blu;

points[512].x = 147.09036453756582;
points[512].y = 104.70825515947467;
points[512].color = giallo;

points[513].x = 155.6454582926978;
points[513].y = 101.70825515947467;
points[513].color = blu;

points[514].x = 148.09036453756582;
points[514].y = 104.70825515947467;
points[514].color = giallo;

points[515].x = 156.6454582926978;
points[515].y = 101.70825515947467;
points[515].color = blu;

points[516].x = 149.09036453756582;
points[516].y = 104.70825515947467;
points[516].color = giallo;

points[517].x = 157.6454582926978;
points[517].y = 101.70825515947467;
points[517].color = blu;

points[518].x = 150.09036453756582;
points[518].y = 104.70825515947467;
points[518].color = giallo;

points[519].x = 158.6454582926978;
points[519].y = 101.70825515947467;
points[519].color = blu;

points[520].x = 151.09036453756582;
points[520].y = 104.70825515947467;
points[520].color = giallo;

points[521].x = 159.6454582926978;
points[521].y = 101.70825515947467;
points[521].color = blu;

points[522].x = 152.09036453756582;
points[522].y = 104.70825515947467;
points[522].color = giallo;

points[523].x = 160.6454582926978;
points[523].y = 101.70825515947467;
points[523].color = blu;

points[524].x = 153.09036453756582;
points[524].y = 104.70825515947467;
points[524].color = giallo;

points[525].x = 161.6454582926978;
points[525].y = 101.70825515947467;
points[525].color = blu;

points[526].x = 154.09036453756582;
points[526].y = 104.70825515947467;
points[526].color = giallo;

points[527].x = 162.6454582926978;
points[527].y = 101.70825515947467;
points[527].color = blu;

points[528].x = 155.09036453756582;
points[528].y = 104.70825515947467;
points[528].color = giallo;

points[529].x = 163.6454582926978;
points[529].y = 101.70825515947467;
points[529].color = blu;

points[530].x = 156.09036453756582;
points[530].y = 104.70825515947467;
points[530].color = giallo;

points[531].x = 164.6454582926978;
points[531].y = 101.70825515947467;
points[531].color = blu;

points[532].x = 157.09036453756582;
points[532].y = 104.70825515947467;
points[532].color = giallo;

points[533].x = 165.6454582926978;
points[533].y = 101.70825515947467;
points[533].color = blu;

points[534].x = 158.09036453756582;
points[534].y = 104.70825515947467;
points[534].color = giallo;

points[535].x = 166.6454582926978;
points[535].y = 101.70825515947467;
points[535].color = blu;

points[536].x = 159.09036453756582;
points[536].y = 104.70825515947467;
points[536].color = giallo;

points[537].x = 167.6454582926978;
points[537].y = 101.70825515947467;
points[537].color = blu;

points[538].x = 160.09036453756582;
points[538].y = 104.70825515947467;
points[538].color = giallo;

points[539].x = 168.6454582926978;
points[539].y = 101.70825515947467;
points[539].color = blu;

points[540].x = 161.09036453756582;
points[540].y = 104.70825515947467;
points[540].color = giallo;

points[541].x = 169.6454582926978;
points[541].y = 101.70825515947467;
points[541].color = blu;

points[542].x = 162.09036453756582;
points[542].y = 104.70825515947467;
points[542].color = giallo;

points[543].x = 170.6454582926978;
points[543].y = 101.70825515947467;
points[543].color = blu;

points[544].x = 163.09036453756582;
points[544].y = 104.70825515947467;
points[544].color = giallo;

points[545].x = 171.6454582926978;
points[545].y = 101.70825515947467;
points[545].color = blu;

points[546].x = 164.09036453756582;
points[546].y = 104.70825515947467;
points[546].color = giallo;

points[547].x = 172.6454582926978;
points[547].y = 101.70825515947467;
points[547].color = blu;

points[548].x = 165.09036453756582;
points[548].y = 104.70825515947467;
points[548].color = giallo;

points[549].x = 173.6454582926978;
points[549].y = 101.70825515947467;
points[549].color = blu;

points[550].x = 166.09036453756582;
points[550].y = 104.70825515947467;
points[550].color = giallo;

points[551].x = 174.6454582926978;
points[551].y = 101.70825515947467;
points[551].color = blu;

points[552].x = 167.09036453756582;
points[552].y = 104.70825515947467;
points[552].color = giallo;

points[553].x = 175.6454582926978;
points[553].y = 101.70825515947467;
points[553].color = blu;

points[554].x = 168.09036453756582;
points[554].y = 104.70825515947467;
points[554].color = giallo;

points[555].x = 176.6454582926978;
points[555].y = 101.70825515947467;
points[555].color = blu;

points[556].x = 169.09036453756582;
points[556].y = 104.70825515947467;
points[556].color = giallo;

points[557].x = 177.6454582926978;
points[557].y = 101.70825515947467;
points[557].color = blu;

points[558].x = 170.09036453756582;
points[558].y = 104.70825515947467;
points[558].color = giallo;

points[559].x = 178.6454582926978;
points[559].y = 101.70825515947467;
points[559].color = blu;

points[560].x = 171.09036453756582;
points[560].y = 104.70825515947467;
points[560].color = giallo;

points[561].x = 179.6454582926978;
points[561].y = 101.70825515947467;
points[561].color = blu;

points[562].x = 172.09036453756582;
points[562].y = 104.70825515947467;
points[562].color = giallo;

points[563].x = 180.6454582926978;
points[563].y = 101.70825515947467;
points[563].color = blu;

points[564].x = 173.09036453756582;
points[564].y = 104.70825515947467;
points[564].color = giallo;

points[565].x = 181.6454582926978;
points[565].y = 101.70825515947467;
points[565].color = blu;

points[566].x = 174.09036453756582;
points[566].y = 104.70825515947467;
points[566].color = giallo;

points[567].x = 182.6454582926978;
points[567].y = 101.70825515947467;
points[567].color = blu;

points[568].x = 175.09036453756582;
points[568].y = 104.70825515947467;
points[568].color = giallo;

points[569].x = 183.6454582926978;
points[569].y = 101.70825515947467;
points[569].color = blu;

points[570].x = 176.09036453756582;
points[570].y = 104.70825515947467;
points[570].color = giallo;

points[571].x = 184.6454582926978;
points[571].y = 101.70825515947467;
points[571].color = blu;

points[572].x = 177.09036453756582;
points[572].y = 104.70825515947467;
points[572].color = giallo;

points[573].x = 185.6454582926978;
points[573].y = 101.70825515947467;
points[573].color = blu;

points[574].x = 178.09036453756582;
points[574].y = 104.70825515947467;
points[574].color = giallo;

points[575].x = 186.6454582926978;
points[575].y = 101.70825515947467;
points[575].color = blu;

points[576].x = 179.09036453756582;
points[576].y = 104.70825515947467;
points[576].color = giallo;

points[577].x = 187.63075349304995;
points[577].y = 101.82082551594746;
points[577].color = blu;

points[578].x = 180.09036453756582;
points[578].y = 104.70825515947467;
points[578].color = giallo;

points[579].x = 188.63075349304995;
points[579].y = 101.82082551594746;
points[579].color = blu;

points[580].x = 181.09036453756582;
points[580].y = 104.70825515947467;
points[580].color = giallo;

points[581].x = 189.63075349304995;
points[581].y = 101.82082551594746;
points[581].color = blu;

points[582].x = 182.09036453756582;
points[582].y = 104.70825515947467;
points[582].color = giallo;

points[583].x = 190.63075349304995;
points[583].y = 101.82082551594746;
points[583].color = blu;

points[584].x = 183.09036453756582;
points[584].y = 104.70825515947467;
points[584].color = giallo;

points[585].x = 191.63075349304995;
points[585].y = 101.82082551594746;
points[585].color = blu;

points[586].x = 184.09036453756582;
points[586].y = 104.70825515947467;
points[586].color = giallo;

points[587].x = 192.63075349304995;
points[587].y = 101.82082551594746;
points[587].color = blu;

points[588].x = 185.09036453756582;
points[588].y = 104.70825515947467;
points[588].color = giallo;

points[589].x = 193.63075349304995;
points[589].y = 101.82082551594746;
points[589].color = blu;

points[590].x = 186.09036453756582;
points[590].y = 104.70825515947467;
points[590].color = giallo;

points[591].x = 194.63075349304995;
points[591].y = 101.82082551594746;
points[591].color = blu;

points[592].x = 187.07565973791796;
points[592].y = 104.82082551594746;
points[592].color = giallo;

points[593].x = 195.63075349304995;
points[593].y = 101.82082551594746;
points[593].color = blu;

points[594].x = 188.07565973791796;
points[594].y = 104.82082551594746;
points[594].color = giallo;

points[595].x = 196.63075349304995;
points[595].y = 101.82082551594746;
points[595].color = blu;

points[596].x = 189.07565973791796;
points[596].y = 104.82082551594746;
points[596].color = giallo;

points[597].x = 197.63075349304995;
points[597].y = 101.82082551594746;
points[597].color = blu;

points[598].x = 190.07565973791796;
points[598].y = 104.82082551594746;
points[598].color = giallo;

points[599].x = 198.63075349304995;
points[599].y = 101.82082551594746;
points[599].color = blu;

points[600].x = 191.07565973791796;
points[600].y = 104.82082551594746;
points[600].color = giallo;

points[601].x = 199.63075349304995;
points[601].y = 101.82082551594746;
points[601].color = blu;

points[602].x = 192.07565973791796;
points[602].y = 104.82082551594746;
points[602].color = giallo;

points[603].x = 200.63075349304995;
points[603].y = 101.82082551594746;
points[603].color = blu;

points[604].x = 193.07565973791796;
points[604].y = 104.82082551594746;
points[604].color = giallo;

points[605].x = 201.63075349304995;
points[605].y = 101.82082551594746;
points[605].color = blu;

points[606].x = 194.07565973791796;
points[606].y = 104.82082551594746;
points[606].color = giallo;

points[607].x = 202.63075349304995;
points[607].y = 101.82082551594746;
points[607].color = blu;

points[608].x = 195.07565973791796;
points[608].y = 104.82082551594746;
points[608].color = giallo;

points[609].x = 203.63075349304995;
points[609].y = 101.82082551594746;
points[609].color = blu;

points[610].x = 196.07565973791796;
points[610].y = 104.82082551594746;
points[610].color = giallo;

points[611].x = 204.63075349304995;
points[611].y = 101.82082551594746;
points[611].color = blu;

points[612].x = 197.07565973791796;
points[612].y = 104.82082551594746;
points[612].color = giallo;

points[613].x = 205.63075349304995;
points[613].y = 101.82082551594746;
points[613].color = blu;

points[614].x = 198.07565973791796;
points[614].y = 104.82082551594746;
points[614].color = giallo;

points[615].x = 206.63075349304995;
points[615].y = 101.82082551594746;
points[615].color = blu;

points[616].x = 199.07565973791796;
points[616].y = 104.82082551594746;
points[616].color = giallo;

points[617].x = 207.63075349304995;
points[617].y = 101.82082551594746;
points[617].color = blu;

points[618].x = 200.07565973791796;
points[618].y = 104.82082551594746;
points[618].color = giallo;

points[619].x = 208.63075349304995;
points[619].y = 101.82082551594746;
points[619].color = blu;

points[620].x = 201.07565973791796;
points[620].y = 104.82082551594746;
points[620].color = giallo;

points[621].x = 209.63075349304995;
points[621].y = 101.82082551594746;
points[621].color = blu;

points[622].x = 202.07565973791796;
points[622].y = 104.82082551594746;
points[622].color = giallo;

points[623].x = 210.63075349304995;
points[623].y = 101.82082551594746;
points[623].color = blu;

points[624].x = 203.07565973791796;
points[624].y = 104.82082551594746;
points[624].color = giallo;

points[625].x = 211.63075349304995;
points[625].y = 101.82082551594746;
points[625].color = blu;

points[626].x = 204.07565973791796;
points[626].y = 104.82082551594746;
points[626].color = giallo;

points[627].x = 212.61604869340223;
points[627].y = 101.93339587242026;
points[627].color = blu;

points[628].x = 205.07565973791796;
points[628].y = 104.82082551594746;
points[628].color = giallo;

points[629].x = 213.61604869340223;
points[629].y = 101.93339587242026;
points[629].color = blu;

points[630].x = 206.07565973791796;
points[630].y = 104.82082551594746;
points[630].color = giallo;

points[631].x = 214.61604869340223;
points[631].y = 101.93339587242026;
points[631].color = blu;

points[632].x = 207.07565973791796;
points[632].y = 104.82082551594746;
points[632].color = giallo;

points[633].x = 215.61604869340223;
points[633].y = 101.93339587242026;
points[633].color = blu;

points[634].x = 208.07565973791796;
points[634].y = 104.82082551594746;
points[634].color = giallo;

points[635].x = 216.61604869340223;
points[635].y = 101.93339587242026;
points[635].color = blu;

points[636].x = 209.07565973791796;
points[636].y = 104.82082551594746;
points[636].color = giallo;

points[637].x = 217.61604869340223;
points[637].y = 101.93339587242026;
points[637].color = blu;

points[638].x = 210.07565973791796;
points[638].y = 104.82082551594746;
points[638].color = giallo;

points[639].x = 218.61604869340223;
points[639].y = 101.93339587242026;
points[639].color = blu;

points[640].x = 211.07565973791796;
points[640].y = 104.82082551594746;
points[640].color = giallo;

points[641].x = 219.61604869340223;
points[641].y = 101.93339587242026;
points[641].color = blu;

points[642].x = 212.06095493827027;
points[642].y = 104.93339587242026;
points[642].color = giallo;

points[643].x = 220.61604869340223;
points[643].y = 101.93339587242026;
points[643].color = blu;

points[644].x = 213.06095493827027;
points[644].y = 104.93339587242026;
points[644].color = giallo;

points[645].x = 221.61604869340223;
points[645].y = 101.93339587242026;
points[645].color = blu;

points[646].x = 214.06095493827027;
points[646].y = 104.93339587242026;
points[646].color = giallo;

points[647].x = 222.61604869340223;
points[647].y = 101.93339587242026;
points[647].color = blu;

points[648].x = 215.06095493827027;
points[648].y = 104.93339587242026;
points[648].color = giallo;

points[649].x = 223.61604869340223;
points[649].y = 101.93339587242026;
points[649].color = blu;

points[650].x = 216.06095493827027;
points[650].y = 104.93339587242026;
points[650].color = giallo;

points[651].x = 224.6013438937544;
points[651].y = 102.04596622889305;
points[651].color = blu;

points[652].x = 217.06095493827027;
points[652].y = 104.93339587242026;
points[652].color = giallo;

points[653].x = 225.6013438937544;
points[653].y = 102.04596622889305;
points[653].color = blu;

points[654].x = 218.06095493827027;
points[654].y = 104.93339587242026;
points[654].color = giallo;

points[655].x = 226.6013438937544;
points[655].y = 102.04596622889305;
points[655].color = blu;

points[656].x = 219.06095493827027;
points[656].y = 104.93339587242026;
points[656].color = giallo;

points[657].x = 227.6013438937544;
points[657].y = 102.04596622889305;
points[657].color = blu;

points[658].x = 220.06095493827027;
points[658].y = 104.93339587242026;
points[658].color = giallo;

points[659].x = 228.6013438937544;
points[659].y = 102.04596622889305;
points[659].color = blu;

points[660].x = 221.06095493827027;
points[660].y = 104.93339587242026;
points[660].color = giallo;

points[661].x = 229.6013438937544;
points[661].y = 102.04596622889305;
points[661].color = blu;

points[662].x = 222.06095493827027;
points[662].y = 104.93339587242026;
points[662].color = giallo;

points[663].x = 230.6013438937544;
points[663].y = 102.04596622889305;
points[663].color = blu;

points[664].x = 223.06095493827027;
points[664].y = 104.93339587242026;
points[664].color = giallo;

points[665].x = 231.6013438937544;
points[665].y = 102.04596622889305;
points[665].color = blu;

points[666].x = 224.04625013862247;
points[666].y = 105.04596622889305;
points[666].color = giallo;

points[667].x = 232.6013438937544;
points[667].y = 102.04596622889305;
points[667].color = blu;

points[668].x = 225.04625013862247;
points[668].y = 105.04596622889305;
points[668].color = giallo;

points[669].x = 233.6013438937544;
points[669].y = 102.04596622889305;
points[669].color = blu;

points[670].x = 226.04625013862247;
points[670].y = 105.04596622889305;
points[670].color = giallo;

points[671].x = 234.6013438937544;
points[671].y = 102.04596622889305;
points[671].color = blu;

points[672].x = 227.04625013862247;
points[672].y = 105.04596622889305;
points[672].color = giallo;

points[673].x = 235.6013438937544;
points[673].y = 102.04596622889305;
points[673].color = blu;

points[674].x = 228.04625013862247;
points[674].y = 105.04596622889305;
points[674].color = giallo;

points[675].x = 236.6013438937544;
points[675].y = 102.04596622889305;
points[675].color = blu;

points[676].x = 229.04625013862247;
points[676].y = 105.04596622889305;
points[676].color = giallo;

points[677].x = 237.6013438937544;
points[677].y = 102.04596622889305;
points[677].color = blu;

points[678].x = 230.04625013862247;
points[678].y = 105.04596622889305;
points[678].color = giallo;

points[679].x = 238.6013438937544;
points[679].y = 102.04596622889305;
points[679].color = blu;

points[680].x = 231.04625013862247;
points[680].y = 105.04596622889305;
points[680].color = giallo;

points[681].x = 239.6013438937544;
points[681].y = 102.04596622889305;
points[681].color = blu;

points[682].x = 232.04625013862247;
points[682].y = 105.04596622889305;
points[682].color = giallo;

points[683].x = 240.6013438937544;
points[683].y = 102.04596622889305;
points[683].color = blu;

points[684].x = 233.04625013862247;
points[684].y = 105.04596622889305;
points[684].color = giallo;

points[685].x = 241.6013438937544;
points[685].y = 102.04596622889305;
points[685].color = blu;

points[686].x = 234.04625013862247;
points[686].y = 105.04596622889305;
points[686].color = giallo;

points[687].x = 242.6013438937544;
points[687].y = 102.04596622889305;
points[687].color = blu;

points[688].x = 235.04625013862247;
points[688].y = 105.04596622889305;
points[688].color = giallo;

points[689].x = 243.6013438937544;
points[689].y = 102.04596622889305;
points[689].color = blu;

points[690].x = 236.04625013862247;
points[690].y = 105.04596622889305;
points[690].color = giallo;

points[691].x = 244.5866390941067;
points[691].y = 102.15853658536585;
points[691].color = blu;

points[692].x = 237.04625013862247;
points[692].y = 105.04596622889305;
points[692].color = giallo;

points[693].x = 245.5866390941067;
points[693].y = 102.15853658536585;
points[693].color = blu;

points[694].x = 238.04625013862247;
points[694].y = 105.04596622889305;
points[694].color = giallo;

points[695].x = 246.5866390941067;
points[695].y = 102.15853658536585;
points[695].color = blu;

points[696].x = 239.04625013862247;
points[696].y = 105.04596622889305;
points[696].color = giallo;

points[697].x = 247.5866390941067;
points[697].y = 102.15853658536585;
points[697].color = blu;

points[698].x = 240.04625013862247;
points[698].y = 105.04596622889305;
points[698].color = giallo;

points[699].x = 248.5866390941067;
points[699].y = 102.15853658536585;
points[699].color = blu;

points[700].x = 241.04625013862247;
points[700].y = 105.04596622889305;
points[700].color = giallo;

points[701].x = 249.5866390941067;
points[701].y = 102.15853658536585;
points[701].color = blu;

points[702].x = 242.04625013862247;
points[702].y = 105.04596622889305;
points[702].color = giallo;

points[703].x = 250.5866390941067;
points[703].y = 102.15853658536585;
points[703].color = blu;

points[704].x = 243.04625013862247;
points[704].y = 105.04596622889305;
points[704].color = giallo;

points[705].x = 251.5866390941067;
points[705].y = 102.15853658536585;
points[705].color = blu;

points[706].x = 244.03154533897478;
points[706].y = 105.15853658536585;
points[706].color = giallo;

points[707].x = 252.5866390941067;
points[707].y = 102.15853658536585;
points[707].color = blu;

points[708].x = 245.03154533897478;
points[708].y = 105.15853658536585;
points[708].color = giallo;

points[709].x = 253.5866390941067;
points[709].y = 102.15853658536585;
points[709].color = blu;

points[710].x = 246.03154533897478;
points[710].y = 105.15853658536585;
points[710].color = giallo;

points[711].x = 254.5866390941067;
points[711].y = 102.15853658536585;
points[711].color = blu;

points[712].x = 247.03154533897478;
points[712].y = 105.15853658536585;
points[712].color = giallo;

points[713].x = 255.5866390941067;
points[713].y = 102.15853658536585;
points[713].color = blu;

points[714].x = 248.03154533897478;
points[714].y = 105.15853658536585;
points[714].color = giallo;

points[715].x = 256.58663909410666;
points[715].y = 102.15853658536585;
points[715].color = blu;

points[716].x = 249.03154533897478;
points[716].y = 105.15853658536585;
points[716].color = giallo;

points[717].x = 257.58663909410666;
points[717].y = 102.15853658536585;
points[717].color = blu;

points[718].x = 250.03154533897478;
points[718].y = 105.15853658536585;
points[718].color = giallo;

points[719].x = 258.58663909410666;
points[719].y = 102.15853658536585;
points[719].color = blu;

points[720].x = 251.03154533897478;
points[720].y = 105.15853658536585;
points[720].color = giallo;

points[721].x = 259.58663909410666;
points[721].y = 102.15853658536585;
points[721].color = blu;

points[722].x = 252.03154533897478;
points[722].y = 105.15853658536585;
points[722].color = giallo;

points[723].x = 260.58663909410666;
points[723].y = 102.15853658536585;
points[723].color = blu;

points[724].x = 253.03154533897478;
points[724].y = 105.15853658536585;
points[724].color = giallo;

points[725].x = 261.58663909410666;
points[725].y = 102.15853658536585;
points[725].color = blu;

points[726].x = 254.03154533897478;
points[726].y = 105.15853658536585;
points[726].color = giallo;

points[727].x = 262.58663909410666;
points[727].y = 102.15853658536585;
points[727].color = blu;

points[728].x = 255.03154533897478;
points[728].y = 105.15853658536585;
points[728].color = giallo;

points[729].x = 263.58663909410666;
points[729].y = 102.15853658536585;
points[729].color = blu;

points[730].x = 256.03154533897475;
points[730].y = 105.15853658536585;
points[730].color = giallo;

points[731].x = 264.58663909410666;
points[731].y = 102.15853658536585;
points[731].color = blu;

points[732].x = 257.03154533897475;
points[732].y = 105.15853658536585;
points[732].color = giallo;

points[733].x = 265.5828708612005;
points[733].y = 102.20415979354227;
points[733].color = blu;

points[734].x = 258.03154533897475;
points[734].y = 105.15853658536585;
points[734].color = giallo;

points[735].x = 266.571934294459;
points[735].y = 102.27110694183865;
points[735].color = blu;

points[736].x = 259.03154533897475;
points[736].y = 105.15853658536585;
points[736].color = giallo;

points[737].x = 267.571934294459;
points[737].y = 102.27110694183865;
points[737].color = blu;

points[738].x = 260.03154533897475;
points[738].y = 105.15853658536585;
points[738].color = giallo;

points[739].x = 268.571934294459;
points[739].y = 102.27110694183865;
points[739].color = blu;

points[740].x = 261.03154533897475;
points[740].y = 105.15853658536585;
points[740].color = giallo;

points[741].x = 269.571934294459;
points[741].y = 102.27110694183865;
points[741].color = blu;

points[742].x = 262.03154533897475;
points[742].y = 105.15853658536585;
points[742].color = giallo;

points[743].x = 270.571934294459;
points[743].y = 102.27110694183865;
points[743].color = blu;

points[744].x = 263.03154533897475;
points[744].y = 105.15853658536585;
points[744].color = giallo;

points[745].x = 271.571934294459;
points[745].y = 102.27110694183865;
points[745].color = blu;

points[746].x = 264.03154533897475;
points[746].y = 105.15853658536585;
points[746].color = giallo;

points[747].x = 272.571934294459;
points[747].y = 102.27110694183865;
points[747].color = blu;

points[748].x = 265.0190747097559;
points[748].y = 105.23887730728399;
points[748].color = giallo;

points[749].x = 273.571934294459;
points[749].y = 102.27110694183865;
points[749].color = blu;

points[750].x = 266.016840539327;
points[750].y = 105.27110694183865;
points[750].color = giallo;

points[751].x = 274.571934294459;
points[751].y = 102.27110694183865;
points[751].color = blu;

points[752].x = 267.016840539327;
points[752].y = 105.27110694183865;
points[752].color = giallo;

points[753].x = 275.571934294459;
points[753].y = 102.27110694183865;
points[753].color = blu;

points[754].x = 268.016840539327;
points[754].y = 105.27110694183865;
points[754].color = giallo;

points[755].x = 276.571934294459;
points[755].y = 102.27110694183865;
points[755].color = blu;

points[756].x = 269.016840539327;
points[756].y = 105.27110694183865;
points[756].color = giallo;

points[757].x = 277.571934294459;
points[757].y = 102.27110694183865;
points[757].color = blu;

points[758].x = 270.016840539327;
points[758].y = 105.27110694183865;
points[758].color = giallo;

points[759].x = 278.5572294948114;
points[759].y = 102.15853658536585;
points[759].color = blu;

points[760].x = 271.016840539327;
points[760].y = 105.27110694183865;
points[760].color = giallo;

points[761].x = 279.5446723811002;
points[761].y = 102.07735734894378;
points[761].color = blu;

points[762].x = 272.016840539327;
points[762].y = 105.27110694183865;
points[762].color = giallo;

points[763].x = 280.5306042716806;
points[763].y = 101.97070443919135;
points[763].color = blu;

points[764].x = 273.016840539327;
points[764].y = 105.27110694183865;
points[764].color = giallo;

points[765].x = 281.51311070729366;
points[765].y = 101.82082551594746;
points[765].color = blu;

points[766].x = 274.016840539327;
points[766].y = 105.27110694183865;
points[766].color = giallo;

points[767].x = 282.49394716788527;
points[767].y = 101.68430592146787;
points[767].color = blu;

points[768].x = 275.016840539327;
points[768].y = 105.27110694183865;
points[768].color = giallo;

points[769].x = 283.46316374731543;
points[769].y = 101.4757516112665;
points[769].color = blu;

points[770].x = 276.016840539327;
points[770].y = 105.27110694183865;
points[770].color = giallo;

points[771].x = 284.43641365064246;
points[771].y = 101.27964415845364;
points[771].color = blu;

points[772].x = 277.016840539327;
points[772].y = 105.27110694183865;
points[772].color = giallo;

points[773].x = 285.31097383516317;
points[773].y = 100.81364725861495;
points[773].color = blu;

points[774].x = 278.016840539327;
points[774].y = 105.27110694183865;
points[774].color = giallo;

points[775].x = 286.13653585753775;
points[775].y = 100.25420865934447;
points[775].color = blu;

points[776].x = 279.0021357396794;
points[776].y = 105.15853658536585;
points[776].color = giallo;

points[777].x = 286.97899298956116;
points[777].y = 99.73025825850787;
points[777].color = blu;

points[778].x = 280.0007620481715;
points[778].y = 105.13521923592708;
points[778].color = giallo;

points[779].x = 287.84038104192126;
points[779].y = 99.22956383016172;
points[779].color = blu;

points[780].x = 280.9864460971233;
points[780].y = 105.0272827967767;
points[780].color = giallo;

points[781].x = 288.6767994093596;
points[781].y = 98.69455215492457;
points[781].color = blu;

points[782].x = 281.95801695216164;
points[782].y = 104.82082551594746;
points[782].color = giallo;

points[783].x = 289.51458095845504;
points[783].y = 98.16249145331926;
points[783].color = blu;

points[784].x = 282.94330045808164;
points[784].y = 104.70727979549507;
points[784].color = giallo;

points[785].x = 290.35704217909347;
points[785].y = 97.6314926892882;
points[785].color = blu;

points[786].x = 283.92504777235416;
points[786].y = 104.55175779499463;
points[786].color = giallo;

points[787].x = 291.2009657089371;
points[787].y = 97.10302681046801;
points[787].color = blu;

points[788].x = 284.89244025082604;
points[788].y = 104.33717941642345;
points[788].color = giallo;

points[789].x = 291.9233811585549;
points[789].y = 96.44442029559625;
points[789].color = blu;

points[790].x = 285.83640581084626;
points[790].y = 104.02741731407897;
points[790].color = giallo;

points[791].x = 292.61106058949093;
points[791].y = 95.72360624911076;
points[791].color = blu;

points[792].x = 286.7051873360772;
points[792].y = 103.54018870688802;
points[792].color = giallo;

points[793].x = 293.2162038849133;
points[793].y = 94.93446097146936;
points[793].color = blu;

points[794].x = 287.5416628546414;
points[794].y = 102.99760773606228;
points[794].color = giallo;

points[795].x = 293.8414158611428;
points[795].y = 94.16200892838273;
points[795].color = blu;

points[796].x = 288.4023679365932;
points[796].y = 102.49628877846435;
points[796].color = giallo;

points[797].x = 294.3723331057552;
points[797].y = 93.3236535458556;
points[797].color = blu;

points[798].x = 289.22579677914507;
points[798].y = 101.94154606901;
points[798].color = giallo;

points[799].x = 294.8636852532652;
points[799].y = 92.45688835646095;
points[799].color = blu;

points[800].x = 290.06395824906474;
points[800].y = 101.41024822683997;
points[800].color = giallo;

points[801].x = 295.3596318981214;
points[801].y = 91.59269450480349;
points[801].color = blu;

points[802].x = 290.9007206875615;
points[802].y = 100.87601623735438;
points[802].color = giallo;

points[803].x = 295.8012070975852;
points[803].y = 90.69869594487709;
points[803].color = blu;

points[804].x = 291.7600994389597;
points[804].y = 100.37169428269941;
points[804].color = giallo;

points[805].x = 296.19833432012604;
points[805].y = 89.78492397244442;
points[805].color = blu;

points[806].x = 292.60610247072026;
points[806].y = 99.84644099779219;
points[806].color = giallo;

points[807].x = 296.529174730273;
points[807].y = 88.84551512151924;
points[807].color = blu;

points[808].x = 293.4300534926315;
points[808].y = 99.28481961360293;
points[808].color = giallo;

points[809].x = 296.8485324029139;
points[809].y = 87.90493204039959;
points[809].color = blu;

points[810].x = 294.13879725655045;
points[810].y = 98.57959355705806;
points[810].color = giallo;

points[811].x = 297.15667444510876;
points[811].y = 86.96075664561947;
points[811].color = blu;

points[812].x = 294.7847148022068;
points[812].y = 97.82441041940167;
points[812].color = giallo;

points[813].x = 297.36971072557697;
points[813].y = 85.98902318902628;
points[813].color = blu;

points[814].x = 295.45009641400947;
points[814].y = 97.08322158692471;
points[814].color = giallo;

points[815].x = 297.60037613708187;
points[815].y = 85.03151057475398;
points[815].color = blu;

points[816].x = 296.0551493982102;
points[816].y = 96.29456367872216;
points[816].color = giallo;

points[817].x = 297.7556757742735;
points[817].y = 84.04973236484678;
points[817].color = blu;

points[818].x = 296.59972715177497;
points[818].y = 95.46351253221263;
points[818].color = giallo;

points[819].x = 297.89085275474844;
points[819].y = 83.06752049936016;
points[819].color = blu;

points[820].x = 297.1558968219485;
points[820].y = 94.64098396889214;
points[820].color = giallo;

points[821].x = 298.0511860062054;
points[821].y = 82.09136657098229;
points[821].color = blu;

points[822].x = 297.65666530955116;
points[822].y = 93.77989571975739;
points[822].color = giallo;

points[823].x = 298.1622889305816;
points[823].y = 81.10604973030709;
points[823].color = blu;

points[824].x = 298.15302251029783;
points[824].y = 92.91594937572654;
points[824].color = giallo;

points[825].x = 298.23705271164835;
points[825].y = 80.11397797732552;
points[825].color = blu;

points[826].x = 298.5536350532677;
points[826].y = 92.00388313951319;
points[826].color = giallo;

points[827].x = 298.2748592870544;
points[827].y = 79.12075452995478;
points[827].color = blu;

points[828].x = 298.9537798701598;
points[828].y = 91.0918260803892;
points[828].color = giallo;

points[829].x = 298.3874296435272;
points[829].y = 78.13545932960267;
points[829].color = blu;

points[830].x = 299.31912570150206;
points[830].y = 90.16498272501185;
points[830].color = giallo;

points[831].x = 298.4422467036082;
points[831].y = 77.1404265871564;
points[831].color = blu;

points[832].x = 299.66453273584284;
points[832].y = 89.23094455549523;
points[832].color = giallo;

points[833].x = 298.5;
points[833].y = 76.15016412925036;
points[833].color = blu;

points[834].x = 299.9170625207771;
points[834].y = 88.26801833661696;
points[834].color = giallo;

points[835].x = 298.5;
points[835].y = 75.15016412925036;
points[835].color = blu;

points[836].x = 300.2259035042344;
points[836].y = 87.32395541702945;
points[836].color = giallo;

points[837].x = 298.5;
points[837].y = 74.15016412925036;
points[837].color = blu;

points[838].x = 300.42265898100885;
points[838].y = 86.35373395856557;
points[838].color = giallo;

points[839].x = 298.39268065939365;
points[839].y = 73.16472231962987;
points[839].color = blu;

points[840].x = 300.6519965596108;
points[840].y = 85.39187585202099;
points[840].color = giallo;

points[841].x = 298.3874296435272;
points[841].y = 72.16486892889807;
points[841].color = blu;

points[842].x = 300.82010040835934;
points[842].y = 84.41215803061691;
points[842].color = giallo;

points[843].x = 298.2748592870544;
points[843].y = 71.17957372854595;
points[843].color = blu;

points[844].x = 300.9680936798774;
points[844].y = 83.43365970372894;
points[844].color = giallo;

points[845].x = 298.2748592870544;
points[845].y = 70.17957372854595;
points[845].color = blu;

points[846].x = 301.1059532548245;
points[846].y = 82.45223823366074;
points[846].color = giallo;

points[847].x = 298.2748592870544;
points[847].y = 69.17957372854595;
points[847].color = blu;

points[848].x = 301.1622889305816;
points[848].y = 81.4574141169385;
points[848].color = giallo;

points[849].x = 298.2748592870544;
points[849].y = 68.17957372854595;
points[849].color = blu;

points[850].x = 301.2748498507416;
points[850].y = 80.47211890211105;
points[850].color = giallo;

points[851].x = 298.16231512307843;
points[851].y = 67.19427848801482;
points[851].color = blu;

points[852].x = 301.2748592870544;
points[852].y = 79.47211891658618;
points[852].color = giallo;

points[853].x = 298.1622889305816;
points[853].y = 66.19427852819364;
points[853].color = blu;

points[854].x = 301.3874296435272;
points[854].y = 78.48682371623407;
points[854].color = giallo;

points[855].x = 298.1622889305816;
points[855].y = 65.19427852819364;
points[855].color = blu;

points[856].x = 301.49827350465085;
points[856].y = 77.50150091353588;
points[856].color = giallo;

points[857].x = 297.9704833385189;
points[857].y = 64.22878565234986;
points[857].color = blu;

points[858].x = 301.5;
points[858].y = 76.50152851588176;
points[858].color = giallo;

points[859].x = 297.54384954961245;
points[859].y = 63.33343233367547;
points[859].color = blu;

points[860].x = 301.5;
points[860].y = 75.50152851588176;
points[860].color = giallo;

points[861].x = 296.9443073772376;
points[861].y = 62.55049746802749;
points[861].color = blu;

points[862].x = 301.5;
points[862].y = 74.50152851588176;
points[862].color = giallo;

points[863].x = 296.1065278544152;
points[863].y = 62.01843264693189;
points[863].color = blu;

points[864].x = 301.5;
points[864].y = 73.50152851588176;
points[864].color = giallo;

points[865].x = 295.1843449616799;
points[865].y = 61.644786237335815;
points[865].color = blu;

points[866].x = 301.3874296435272;
points[866].y = 72.51623331552949;
points[866].color = giallo;

points[867].x = 294.2378767366966;
points[867].y = 61.348325920460454;
points[867].color = blu;

points[868].x = 301.3874296435272;
points[868].y = 71.51623331552949;
points[868].color = giallo;

points[869].x = 293.2734997290674;
points[869].y = 61.109045537325585;
points[869].color = blu;

points[870].x = 301.2748592870544;
points[870].y = 70.53093811517735;
points[870].color = giallo;

points[871].x = 292.3080023380846;
points[871].y = 60.910883389265095;
points[871].color = blu;

points[872].x = 301.2748592870544;
points[872].y = 69.53093811517735;
points[872].color = giallo;

points[873].x = 291.33625844667336;
points[873].y = 60.697889315891565;
points[873].color = blu;

points[874].x = 301.2748592870544;
points[874].y = 68.53093811517735;
points[874].color = giallo;

points[875].x = 290.40396585437776;
points[875].y = 60.34702339914957;
points[875].color = blu;

points[876].x = 301.2748592870544;
points[876].y = 67.53093811517735;
points[876].color = giallo;

points[877].x = 289.58361380141116;
points[877].y = 59.791238084933276;
points[877].color = blu;

points[878].x = 301.1622889305816;
points[878].y = 66.54564291482504;
points[878].color = giallo;

points[879].x = 288.9526754078341;
points[879].y = 59.02434664510502;
points[879].color = blu;

points[880].x = 301.1622889305816;
points[880].y = 65.54564291482504;
points[880].color = giallo;

points[881].x = 288.81894934333957;
points[881].y = 58.04342718399097;
points[881].color = blu;

points[882].x = 301.1614156795989;
points[882].y = 64.54565288606075;
points[882].color = giallo;

points[883].x = 288.81894934333957;
points[883].y = 57.04342718399097;
points[883].color = blu;

points[884].x = 301.00651790210424;
points[884].y = 63.56381884090544;
points[884].color = giallo;

points[885].x = 288.81894934333957;
points[885].y = 56.04342718399097;
points[885].color = blu;

points[886].x = 300.63997544919766;
points[886].y = 62.63743868082297;
points[886].color = giallo;

points[887].x = 288.9315196998124;
points[887].y = 55.05813198363868;
points[887].color = blu;

points[888].x = 300.16817017941844;
points[888].y = 61.75853077783483;
points[888].color = giallo;

points[889].x = 289.04409005628514;
points[889].y = 54.072836783286576;
points[889].color = blu;

points[890].x = 299.5975604803192;
points[890].y = 60.947159461337435;
points[890].color = giallo;

points[891].x = 289.0446741889979;
points[891].y = 53.07284224912361;
points[891].color = blu;

points[892].x = 298.8904536991326;
points[892].y = 60.2400526801508;
points[892].color = giallo;

points[893].x = 289.15666041275796;
points[893].y = 52.08754158293426;
points[893].color = blu;

points[894].x = 298.1109870190373;
points[894].y = 59.62103144006649;
points[894].color = giallo;

points[895].x = 289.2612668846838;
points[895].y = 51.10070270723686;
points[895].color = blu;

points[896].x = 297.22068693075073;
points[896].y = 59.16836306462701;
points[896].color = giallo;

points[897].x = 289.3834713454201;
points[897].y = 50.11698187803585;
points[897].color = blu;

points[898].x = 296.3026777552393;
points[898].y = 58.781190311791846;
points[898].color = giallo;

points[899].x = 289.49437148217635;
points[899].y = 49.13166037045234;
points[899].color = blu;

points[900].x = 295.3600444714606;
points[900].y = 58.46858180820682;
points[900].color = giallo;

points[901].x = 289.60694183864916;
points[901].y = 48.146365170100076;
points[901].color = blu;

points[902].x = 294.38805462712236;
points[902].y = 58.25619361993077;
points[902].color = giallo;

points[903].x = 289.7195121951219;
points[903].y = 47.16106996974794;
points[903].color = blu;

points[904].x = 293.43298737736177;
points[904].y = 58.01323787697922;
points[904].color = giallo;

points[905].x = 289.84340948951365;
points[905].y = 46.176243462960734;
points[905].color = blu;

points[906].x = 292.4589394773708;
points[906].y = 57.81013996199457;
points[906].color = giallo;

points[907].x = 289.95282128973383;
points[907].y = 45.19076812061383;
points[907].color = blu;

points[908].x = 291.81894934333957;
points[908].y = 57.27665196829939;
points[908].color = giallo;

points[909].x = 290.0572232645403;
points[909].y = 44.205188757265965;
points[909].color = blu;

points[910].x = 291.81894934333957;
points[910].y = 56.27665196829939;
points[910].color = giallo;

points[911].x = 290.1697936210131;
points[911].y = 43.21989355691369;
points[911].color = blu;

points[912].x = 291.9315196998124;
points[912].y = 55.29135676794708;
points[912].color = giallo;

points[913].x = 290.26528009969604;
points[913].y = 42.23136124749041;
points[913].color = blu;

points[914].x = 292.04409005628514;
points[914].y = 54.306061567594966;
points[914].color = giallo;

points[915].x = 290.3949343339587;
points[915].y = 41.24930754478403;
points[915].color = blu;

points[916].x = 292.1228844954351;
points[916].y = 53.31836882990533;
points[916].color = giallo;

points[917].x = 290.4019128818186;
points[917].y = 40.249532129893765;
points[917].color = blu;

points[918].x = 292.15666041275796;
points[918].y = 52.320766367242655;
points[918].color = giallo;

points[919].x = 290.5075046904315;
points[919].y = 39.264012344431706;
points[919].color = blu;

points[920].x = 292.2692307692308;
points[920].y = 51.33547116689038;
points[920].color = giallo;

points[921].x = 290.5075046904315;
points[921].y = 38.264012344431706;
points[921].color = blu;

points[922].x = 292.46644941445896;
points[922].y = 50.36308403816956;
points[922].color = giallo;

points[923].x = 290.5075046904315;
points[923].y = 37.264012344431706;
points[923].color = blu;

points[924].x = 292.545654352898;
points[924].y = 49.373713768838;
points[924].color = giallo;

points[925].x = 290.5075046904315;
points[925].y = 36.264012344431706;
points[925].color = blu;

points[926].x = 292.6122656954067;
points[926].y = 48.38062949596391;
points[926].color = giallo;

points[927].x = 290.5075046904315;
points[927].y = 35.264012344431706;
points[927].color = blu;

points[928].x = 292.7195121951219;
points[928].y = 47.39429475405629;
points[928].color = giallo;

points[929].x = 290.3949343339587;
points[929].y = 34.278717144079394;
points[929].color = blu;

points[930].x = 292.9343299305706;
points[930].y = 46.42330471327574;
points[930].color = giallo;

points[931].x = 290.2979678136706;
points[931].y = 33.2926703695408;
points[931].color = blu;

points[932].x = 293.04338361104806;
points[932].y = 45.43778607221269;
points[932].color = giallo;

points[933].x = 290.1697936210131;
points[933].y = 32.30915298127735;
points[933].color = blu;

points[934].x = 293.0629945021328;
points[934].y = 44.439538528823455;
points[934].color = giallo;

points[935].x = 289.7760939826253;
points[935].y = 31.404923608269872;
points[935].color = blu;

points[936].x = 293.1697936210131;
points[936].y = 43.45311834122201;
points[936].color = giallo;

points[937].x = 289.246939268987;
points[937].y = 30.56563143623142;
points[937].color = blu;

points[938].x = 293.3419144520169;
points[938].y = 42.47780785388618;
points[938].color = giallo;

points[939].x = 288.498175740335;
points[939].y = 29.95638735095623;
points[939].color = blu;

points[940].x = 293.3949343339587;
points[940].y = 41.48253232909238;
points[940].color = giallo;

points[941].x = 287.56611698693314;
points[941].y = 29.60497099288824;
points[941].color = blu;

points[942].x = 293.49199966315416;
points[942].y = 40.496492690758046;
points[942].color = giallo;

points[943].x = 286.6464925145883;
points[943].y = 29.229582824575363;
points[943].color = blu;

points[944].x = 293.5075046904315;
points[944].y = 39.4972371287401;
points[944].color = giallo;

points[945].x = 285.72821341265677;
points[945].y = 28.84852108854097;
points[945].color = blu;

points[946].x = 293.5075046904315;
points[946].y = 38.4972371287401;
points[946].color = giallo;

points[947].x = 284.8123776628952;
points[947].y = 28.461747092788936;
points[947].color = blu;

points[948].x = 293.5075046904315;
points[948].y = 37.4972371287401;
points[948].color = giallo;

points[949].x = 283.8783886264971;
points[949].y = 28.116129332300808;
points[949].color = blu;

points[950].x = 293.5075046904315;
points[950].y = 36.4972371287401;
points[950].color = giallo;

points[951].x = 282.93331579487614;
points[951].y = 27.79589897526838;
points[951].color = blu;

points[952].x = 293.5075046904315;
points[952].y = 35.4972371287401;
points[952].color = giallo;

points[953].x = 281.99983701878557;
points[953].y = 27.448994200841707;
points[953].color = blu;

points[954].x = 293.49943188253076;
points[954].y = 34.497516402648074;
points[954].color = giallo;

points[955].x = 281.0658381314481;
points[955].y = 27.103418494209986;
points[955].color = blu;

points[956].x = 293.3949343339587;
points[956].y = 33.51194192838777;
points[956].color = giallo;

points[957].x = 280.1320270758717;
points[957].y = 26.757083169290294;
points[957].color = blu;

points[958].x = 293.26747193246786;
points[958].y = 32.52837319658326;
points[958].color = giallo;

points[959].x = 279.1762035921289;
points[959].y = 26.475509879489003;
points[959].color = blu;

points[960].x = 293.1406253565446;
points[960].y = 31.544300764334125;
points[960].color = giallo;

points[961].x = 278.2439345317457;
points[961].y = 26.1245875483344;
points[961].color = blu;

points[962].x = 292.834565619818;
points[962].y = 30.599196424968564;
points[962].color = giallo;

points[963].x = 277.308537044332;
points[963].y = 25.78590289520651;
points[963].color = blu;

points[964].x = 292.34845404376665;
points[964].y = 29.72991346020026;
points[964].color = giallo;

points[965].x = 276.36761057760407;
points[965].y = 25.46473924483484;
points[965].color = blu;

points[966].x = 291.8100691089984;
points[966].y = 28.895274864402865;
points[966].color = giallo;

points[967].x = 275.40629674350674;
points[967].y = 25.207494419455294;
points[967].color = blu;

points[968].x = 291.1929656814416;
points[968].y = 28.114797307863203;
points[968].color = giallo;

points[969].x = 274.4670214654282;
points[969].y = 24.88184049418968;
points[969].color = blu;

points[970].x = 290.40867158453864;
points[970].y = 27.509079534158627;
points[970].color = giallo;

points[971].x = 273.52795538889217;
points[971].y = 24.55369422413878;
points[971].color = blu;

points[972].x = 289.5150196530347;
points[972].y = 27.06291910397676;
points[972].color = giallo;

points[973].x = 272.56508313463723;
points[973].y = 24.301582158485303;
points[973].color = blu;

points[974].x = 288.60619042637194;
points[974].y = 26.659997877176764;
points[974].color = giallo;

points[975].x = 271.6255745664041;
points[975].y = 23.977279360717272;
points[975].color = blu;

points[976].x = 287.6705269234445;
points[976].y = 26.31908952229581;
points[976].color = giallo;

points[977].x = 270.6866430557812;
points[977].y = 23.649288821344662;
points[977].color = blu;

points[978].x = 286.74417274587057;
points[978].y = 25.956882970107166;
points[978].color = giallo;

points[979].x = 269.72397204459674;
points[979].y = 23.395177007501665;
points[979].color = blu;

points[980].x = 285.8276107692476;
points[980].y = 25.570965741063418;
points[980].color = giallo;

points[981].x = 268.77977939408754;
points[981].y = 23.08714015089282;
points[981].color = blu;

points[982].x = 284.89424677816567;
points[982].y = 25.22306094417156;
points[982].color = giallo;

points[983].x = 267.83522967981366;
points[983].y = 22.776766156042648;
points[983].color = blu;

points[984].x = 283.9527261739264;
points[984].y = 24.897375390972442;
points[984].color = giallo;

points[985].x = 266.877277697782;
points[985].y = 22.51053790936544;
points[985].color = blu;

points[986].x = 283.015024608103;
points[986].y = 24.55837933112742;
points[986].color = giallo;

points[987].x = 265.938205765106;
points[987].y = 22.183564291998515;
points[987].color = blu;

points[988].x = 282.08167350053196;
points[988].y = 24.21043342535507;
points[988].color = giallo;

points[989].x = 264.980012391613;
points[989].y = 21.91553396740547;
points[989].color = blu;

points[990].x = 281.14809777201583;
points[990].y = 23.863237083589517;
points[990].color = giallo;

points[991].x = 264.03426192336184;
points[991].y = 21.610379557694156;
points[991].color = blu;

points[992].x = 280.1923560899893;
points[992].y = 23.58339593826539;
points[992].color = giallo;

points[993].x = 263.0900510509257;
points[993].y = 21.301824842164002;
points[993].color = blu;

points[994].x = 279.2598513239475;
points[994].y = 23.233054157540707;
points[994].color = giallo;

points[995].x = 262.14143171519066;
points[995].y = 21.002179566327843;
points[995].color = blu;

points[996].x = 278.32706711932866;
points[996].y = 22.883443276808634;
points[996].color = giallo;

points[997].x = 261.1947620109723;
points[997].y = 20.69918165011672;
points[997].color = blu;

points[998].x = 277.38378146914573;
points[998].y = 22.57259683501446;
points[998].color = giallo;

points[999].x = 260.2552469880813;
points[999].y = 20.37491412121861;
points[999].color = blu;

points[1000].x = 276.4284556444809;
points[1000].y = 22.289563925878923;
points[1000].color = giallo;

points[1001].x = 259.3181840274718;
points[1001].y = 20.039712526082425;
points[1001].color = blu;

points[1002].x = 275.48683838191215;
points[1002].y = 21.973894755613358;
points[1002].color = giallo;

points[1003].x = 258.3607325317971;
points[1003].y = 19.764669184441967;
points[1003].color = blu;

points[1004].x = 274.5440332076659;
points[1004].y = 21.66176413313987;
points[1004].color = giallo;

points[1005].x = 257.41968865387065;
points[1005].y = 19.44704549949761;
points[1005].color = blu;

points[1006].x = 273.58767481630804;
points[1006].y = 21.381806596514764;
points[1006].color = giallo;

points[1007].x = 256.49852478468773;
points[1007].y = 19.069132789400932;
points[1007].color = blu;

points[1008].x = 272.64644938840433;
points[1008].y = 21.06482348639004;
points[1008].color = giallo;

points[1009].x = 255.5473496170206;
points[1009].y = 18.775266014550233;
points[1009].color = blu;

points[1010].x = 271.7041010825958;
points[1010].y = 20.751400095924485;
points[1010].color = giallo;

points[1011].x = 254.5812755393455;
points[1011].y = 18.582780617653835;
points[1011].color = blu;

points[1012].x = 270.7392979842163;
points[1012].y = 20.504055803809834;
points[1012].color = giallo;

points[1013].x = 253.5991795287549;
points[1013].y = 18.435626731258566;
points[1013].color = blu;

points[1014].x = 269.79683694074726;
points[1014].y = 20.190958008260353;
points[1014].color = giallo;

points[1015].x = 252.62329298720715;
points[1015].y = 18.263323974560944;
points[1015].color = blu;

points[1016].x = 268.8530607839365;
points[1016].y = 19.88135347852476;
points[1016].color = giallo;

points[1017].x = 251.63975768508828;
points[1017].y = 18.366791744840526;
points[1017].color = blu;

points[1018].x = 267.8987945286431;
points[1018].y = 19.59534459259746;
points[1018].color = giallo;

points[1019].x = 250.65446248473603;
points[1019].y = 18.47936210131332;
points[1019].color = blu;

points[1020].x = 266.95677256752043;
points[1020].y = 19.28094788352377;
points[1020].color = giallo;

points[1021].x = 249.67701204663467;
points[1021].y = 18.636582548505125;
points[1021].color = blu;

points[1022].x = 266.0002467616995;
points[1022].y = 19.015387328512332;
points[1022].color = giallo;

points[1023].x = 248.72378133936422;
points[1023].y = 18.903081366430243;
points[1023].color = blu;

points[1024].x = 265.0495420692414;
points[1024].y = 18.720413948305797;
points[1024].color = giallo;

points[1025].x = 247.76229459639532;
points[1025].y = 19.159784488387693;
points[1025].color = blu;

points[1026].x = 264.1061448535036;
points[1026].y = 18.40985534720073;
points[1026].color = giallo;

points[1027].x = 246.82970075086143;
points[1027].y = 19.509898749685362;
points[1027].color = blu;

points[1028].x = 263.161307043241;
points[1028].y = 18.102740160462723;
points[1028].color = giallo;

points[1029].x = 245.92580258575668;
points[1029].y = 19.92234616376087;
points[1029].color = blu;

points[1030].x = 262.20935026976156;
points[1030].y = 17.81075950006074;
points[1030].color = giallo;

points[1031].x = 245.0526413546547;
points[1031].y = 20.402490017079913;
points[1031].color = blu;

points[1032].x = 261.27614632271417;
points[1032].y = 17.462353601975796;
points[1032].color = giallo;

points[1033].x = 244.2227683796769;
points[1033].y = 20.94765476434267;
points[1033].color = blu;

points[1034].x = 260.3338095767193;
points[1034].y = 17.148896271467464;
points[1034].color = giallo;

points[1035].x = 243.4005538079978;
points[1035].y = 21.505473407065516;
points[1035].color = blu;

points[1036].x = 259.37645042594266;
points[1036].y = 16.872113958723105;
points[1036].color = giallo;

points[1037].x = 242.6625574959104;
points[1037].y = 22.172643602223395;
points[1037].color = blu;

points[1038].x = 258.44240922997517;
points[1038].y = 16.526720210966776;
points[1038].color = giallo;

points[1039].x = 241.9085258401991;
points[1039].y = 22.820674428313612;
points[1039].color = blu;

points[1040].x = 257.5134764770088;
points[1040].y = 16.17997583475274;
points[1040].color = giallo;

points[1041].x = 241.29044675450177;
points[1041].y = 23.597700046514053;
points[1041].color = blu;

points[1042].x = 256.5675034064426;
points[1042].y = 15.875278117410906;
points[1042].color = giallo;

points[1043].x = 240.68165300468982;
points[1043].y = 24.38525833324131;
points[1043].color = blu;

points[1044].x = 255.61037889780982;
points[1044].y = 15.603797885548996;
points[1044].color = giallo;

points[1045].x = 240.1416224201956;
points[1045].y = 25.22225098075914;
points[1045].color = blu;

points[1046].x = 254.62558191452865;
points[1046].y = 15.47936210131332;
points[1046].color = giallo;

points[1047].x = 239.65410620788893;
points[1047].y = 26.090900176549628;
points[1047].color = blu;

points[1048].x = 253.6402867141758;
points[1048].y = 15.366791744840526;
points[1048].color = giallo;

points[1049].x = 239.2509746331834;
points[1049].y = 26.99955372199443;
points[1049].color = blu;

points[1050].x = 252.65499151382295;
points[1050].y = 15.254221388367728;
points[1050].color = giallo;

points[1051].x = 238.86768044973977;
points[1051].y = 27.918360267303704;
points[1051].color = blu;

points[1052].x = 251.6553234116334;
points[1052].y = 15.263278008766903;
points[1052].color = giallo;

points[1053].x = 238.54598355584008;
points[1053].y = 28.858391234663205;
points[1053].color = blu;

points[1054].x = 250.67084709081973;
points[1054].y = 15.387518322418897;
points[1054].color = giallo;

points[1055].x = 238.34231801649108;
points[1055].y = 29.83053646318515;
points[1055].color = blu;

points[1056].x = 249.6871640387711;
points[1056].y = 15.516479913839666;
points[1056].color = giallo;

points[1057].x = 238.12288930581613;
points[1057].y = 30.80026689516175;
points[1057].color = blu;

points[1058].x = 248.71383718505038;
points[1058].y = 15.705989527921787;
points[1058].color = giallo;

points[1059].x = 238.1210534457995;
points[1059].y = 31.800236686761945;
points[1059].color = blu;

points[1060].x = 247.7595479713127;
points[1060].y = 15.969983080573284;
points[1060].color = giallo;

points[1061].x = 238.12288930581613;
points[1061].y = 32.78615937658316;
points[1061].color = blu;

points[1062].x = 246.80195970965642;
points[1062].y = 16.246008692232436;
points[1062].color = giallo;

points[1063].x = 238.20174435937193;
points[1063].y = 33.77384567449514;
points[1063].color = blu;

points[1064].x = 245.8692743723797;
points[1064].y = 16.59588414594481;
points[1064].color = giallo;

points[1065].x = 238.43970613297785;
points[1065].y = 34.741157458539334;
points[1065].color = blu;

points[1066].x = 244.96966896238703;
points[1066].y = 17.00685057242231;
points[1066].color = giallo;

points[1067].x = 238.659234260908;
points[1067].y = 35.71069204648269;
points[1067].color = blu;

points[1068].x = 244.09446617562645;
points[1068].y = 17.479186789777355;
points[1068].color = giallo;

points[1069].x = 238.90631287090585;
points[1069].y = 36.67483174154073;
points[1069].color = blu;

points[1070].x = 243.21817510725535;
points[1070].y = 17.954881192903315;
points[1070].color = giallo;

points[1071].x = 239.1579363113009;
points[1071].y = 37.62902599151229;
points[1071].color = blu;

points[1072].x = 242.3895581855024;
points[1072].y = 18.501831443090452;
points[1072].color = giallo;

points[1073].x = 239.4682786508766;
points[1073].y = 38.572507529359356;
points[1073].color = blu;

points[1074].x = 241.56332713485628;
points[1074].y = 19.053940488506235;
points[1074].color = giallo;

points[1075].x = 239.90582226488687;
points[1075].y = 39.462414636825926;
points[1075].color = blu;

points[1076].x = 240.8314072759169;
points[1076].y = 19.72974868095153;
points[1076].color = giallo;

points[1077].x = 240.40238083782162;
points[1077].y = 40.326238327749714;
points[1077].color = blu;

points[1078].x = 240.0903171674198;
points[1078].y = 20.396242413973646;
points[1078].color = giallo;

points[1079].x = 240.97838091936686;
points[1079].y = 41.13777145174845;
points[1079].color = blu;

points[1080].x = 239.39013053051772;
points[1080].y = 21.109770042029005;
points[1080].color = giallo;

points[1081].x = 241.60746553694352;
points[1081].y = 41.906407857549;
points[1081].color = blu;

points[1082].x = 238.75869895070494;
points[1082].y = 21.878333287877332;
points[1082].color = giallo;

points[1083].x = 242.36469576564798;
points[1083].y = 42.55106772978066;
points[1083].color = blu;

points[1084].x = 238.16197833006356;
points[1084].y = 22.673204487200046;
points[1084].color = giallo;

points[1085].x = 243.16322275453643;
points[1085].y = 43.142920897153765;
points[1085].color = blu;

points[1086].x = 237.60352998669745;
points[1086].y = 23.499481545665414;
points[1086].color = giallo;

points[1087].x = 243.9761577369501;
points[1087].y = 43.7123758289185;
points[1087].color = blu;

points[1088].x = 237.10829928119293;
points[1088].y = 24.364099724134874;
points[1088].color = giallo;

points[1089].x = 244.82662972095247;
points[1089].y = 44.225992764985826;
points[1089].color = blu;

points[1090].x = 236.61767742926062;
points[1090].y = 25.231241527226103;
points[1090].color = giallo;

points[1091].x = 245.71336724516962;
points[1091].y = 44.66956618194307;
points[1091].color = blu;

points[1092].x = 236.26564479669537;
points[1092].y = 26.163024877580558;
points[1092].color = giallo;

points[1093].x = 246.60003703399042;
points[1093].y = 45.11328094243528;
points[1093].color = blu;

points[1094].x = 235.9143280635451;
points[1094].y = 27.095126698996207;
points[1094].color = giallo;

points[1095].x = 247.50918837528883;
points[1095].y = 45.519769235149056;
points[1095].color = blu;

points[1096].x = 235.58683823335264;
points[1096].y = 28.03437227057466;
points[1096].color = giallo;

points[1097].x = 248.42640567016338;
points[1097].y = 45.908186769240444;
points[1097].color = blu;

points[1098].x = 235.36819715000965;
points[1098].y = 29.004421088194842;
points[1098].color = giallo;

points[1099].x = 249.3487473522067;
points[1099].y = 46.28196851950726;
points[1099].color = blu;

points[1100].x = 235.1508017047019;
points[1100].y = 29.974676431345358;
points[1100].color = giallo;

points[1101].x = 250.2870623170302;
points[1101].y = 46.61954036834058;
points[1101].color = blu;

points[1102].x = 235.12288930581613;
points[1102].y = 30.97287625207892;
points[1102].color = giallo;

points[1103].x = 251.21313549352436;
points[1103].y = 46.979129309071446;
points[1103].color = blu;

points[1104].x = 235.01031894934334;
points[1104].y = 31.958171452433223;
points[1104].color = giallo;

points[1105].x = 252.1457878399207;
points[1105].y = 47.3290912770165;
points[1105].color = blu;

points[1106].x = 235.0755870674862;
points[1106].y = 32.95171096628202;
points[1106].color = giallo;

points[1107].x = 253.06396738173146;
points[1107].y = 47.71435497504615;
points[1107].color = blu;

points[1108].x = 235.12288930581613;
points[1108].y = 33.94346665278745;
points[1108].color = giallo;

points[1109].x = 253.98404036382505;
points[1109].y = 48.08959668345149;
points[1109].color = blu;

points[1110].x = 235.3180199341351;
points[1110].y = 34.918528128914616;
points[1110].color = giallo;

points[1111].x = 254.90177569337047;
points[1111].y = 48.477483075222025;
points[1111].color = blu;

points[1112].x = 235.52950520380787;
points[1112].y = 35.89028467218198;
points[1112].color = giallo;

points[1113].x = 255.71870900627397;
points[1113].y = 49.03828922556612;
points[1113].color = blu;

points[1114].x = 235.80766401932908;
points[1114].y = 36.84637639983423;
points[1114].color = giallo;

points[1115].x = 256.30478233799505;
points[1115].y = 49.83736617770722;
points[1115].color = blu;

points[1116].x = 236.02351007275774;
points[1116].y = 37.81730630174579;
points[1116].color = giallo;

points[1117].x = 256.7004928706968;
points[1117].y = 50.75179767072948;
points[1117].color = blu;

points[1118].x = 236.3025666439721;
points[1118].y = 38.76647461225106;
points[1118].color = giallo;

points[1119].x = 257.10384020251115;
points[1119].y = 51.66035401225896;
points[1119].color = blu;

points[1120].x = 236.62770225991247;
points[1120].y = 39.70510250543097;
points[1120].color = giallo;

points[1121].x = 257.45644635838875;
points[1121].y = 52.5918678439802;
points[1121].color = blu;

points[1122].x = 237.03203378555995;
points[1122].y = 40.615188980149135;
points[1122].color = giallo;

points[1123].x = 257.84715521334454;
points[1123].y = 53.50847624737809;
points[1123].color = blu;

points[1124].x = 237.51784103217926;
points[1124].y = 41.484605523881704;
points[1124].color = giallo;

points[1125].x = 258.22572219599135;
points[1125].y = 54.42944119611899;
points[1125].color = blu;

points[1126].x = 238.057597804923;
points[1126].y = 42.32266725126764;
points[1126].color = giallo;

points[1127].x = 258.57160091259766;
points[1127].y = 55.363367436253114;
points[1127].color = blu;

points[1128].x = 238.6746802581362;
points[1128].y = 43.10369290938813;
points[1128].color = giallo;

points[1129].x = 258.9156790119509;
points[1129].y = 56.29781989259475;
points[1129].color = blu;

points[1130].x = 239.31934013036778;
points[1130].y = 43.86092313809252;
points[1130].color = giallo;

points[1131].x = 259.2677807959039;
points[1131].y = 57.22957132283501;
points[1131].color = blu;

points[1132].x = 240.0418526928627;
points[1132].y = 44.55070272299198;
points[1132].color = giallo;

points[1133].x = 259.6191683966171;
points[1133].y = 58.161642508089464;
points[1133].color = blu;

points[1134].x = 240.7945384513137;
points[1134].y = 45.200709674404244;
points[1134].color = giallo;

points[1135].x = 260.0047842175479;
points[1135].y = 59.08023343696248;
points[1135].color = blu;

points[1136].x = 241.59284425621664;
points[1136].y = 45.7930163793767;
points[1136].color = giallo;

points[1137].x = 260.3524456892483;
points[1137].y = 60.01367205901938;
points[1137].color = blu;

points[1138].x = 242.4113797152935;
points[1138].y = 46.35568403677389;
points[1138].color = giallo;

points[1139].x = 260.6804251319512;
points[1139].y = 60.95423877337237;
points[1139].color = blu;

points[1140].x = 243.27390097579698;
points[1140].y = 46.854312334236695;
points[1140].color = giallo;

points[1141].x = 261.0173910891184;
points[1141].y = 61.892813047111815;
points[1141].color = blu;

points[1142].x = 244.13915851451694;
points[1142].y = 47.34843524513718;
points[1142].color = giallo;

points[1143].x = 261.36508356044226;
points[1143].y = 62.82624226553607;
points[1143].color = blu;

points[1144].x = 245.02617253364602;
points[1144].y = 47.791466328560325;
points[1144].color = giallo;

points[1145].x = 261.7120209845624;
points[1145].y = 63.7598913047959;
points[1145].color = blu;

points[1146].x = 245.93940028720527;
points[1146].y = 48.189733154582974;
points[1146].color = giallo;

points[1147].x = 262.0464982435745;
points[1147].y = 64.69787224058508;
points[1147].color = blu;

points[1148].x = 246.8505311584358;
points[1148].y = 48.59179020808363;
points[1148].color = giallo;

points[1149].x = 262.377721431636;
points[1149].y = 65.63881247205222;
points[1149].color = blu;

points[1150].x = 247.78175385214612;
points[1150].y = 48.944986674117224;
points[1150].color = giallo;

points[1151].x = 262.7247004478452;
points[1151].y = 66.57244986373848;
points[1151].color = blu;

points[1152].x = 248.71332129633458;
points[1152].y = 49.29748068139785;
points[1152].color = giallo;

points[1153].x = 263.0709236497473;
points[1153].y = 67.50628985407785;
points[1153].color = blu;

points[1154].x = 249.628433060973;
points[1154].y = 49.687100406552396;
points[1154].color = giallo;

points[1155].x = 263.4119524803605;
points[1155].y = 68.44182441603448;
points[1155].color = blu;

points[1156].x = 250.56148451827048;
points[1156].y = 50.03596122373805;
points[1156].color = giallo;

points[1157].x = 263.73737991112796;
points[1157].y = 69.38500842268084;
points[1157].color = blu;

points[1158].x = 251.48617725083693;
points[1158].y = 50.403246578017594;
points[1158].color = giallo;

points[1159].x = 264.08363854701565;
points[1159].y = 70.31883935878848;
points[1159].color = blu;

points[1160].x = 252.4235059261229;
points[1160].y = 50.741312470054744;
points[1160].color = giallo;

points[1161].x = 264.44785099040206;
points[1161].y = 71.24155956479339;
points[1161].color = blu;

points[1162].x = 253.34777340163672;
points[1162].y = 51.10726993039957;
points[1162].color = giallo;

points[1163].x = 264.4490395739568;
points[1163].y = 72.24001174070702;
points[1163].color = blu;

points[1164].x = 253.87587724105248;
points[1164].y = 51.91111602490017;
points[1164].color = giallo;

points[1165].x = 264.05344625483036;
points[1165].y = 73.14917277317618;
points[1165].color = blu;

points[1166].x = 254.23019732773975;
points[1166].y = 52.84174485832261;
points[1166].color = giallo;

points[1167].x = 263.42055153869137;
points[1167].y = 73.9165286547745;
points[1167].color = blu;

points[1168].x = 254.62774963054758;
points[1168].y = 53.753100127949786;
points[1168].color = giallo;

points[1169].x = 262.6450106587314;
points[1169].y = 74.5347828511228;
points[1169].color = blu;

points[1170].x = 254.99994111106486;
points[1170].y = 54.67583654761736;
points[1170].color = giallo;

points[1171].x = 261.68776297818465;
points[1171].y = 74.75692524259483;
points[1171].color = blu;

points[1172].x = 255.3841883744348;
points[1172].y = 55.59492158816201;
points[1172].color = giallo;

points[1173].x = 260.787058435421;
points[1173].y = 74.35862830051025;
points[1173].color = blu;

points[1174].x = 255.7352408857799;
points[1174].y = 56.52713617374844;
points[1174].color = giallo;

points[1175].x = 259.94545412077866;
points[1175].y = 73.82601394324803;
points[1175].color = blu;

points[1176].x = 256.0870186025052;
points[1176].y = 57.45903514757312;
points[1176].color = giallo;

points[1177].x = 259.0787233114534;
points[1177].y = 73.33459602617816;
points[1177].color = blu;

points[1178].x = 256.439506474205;
points[1178].y = 58.39060551674459;
points[1178].color = giallo;

points[1179].x = 258.2119592656024;
points[1179].y = 72.84365711554193;
points[1179].color = blu;

points[1180].x = 256.7798238616406;
points[1180].y = 59.326398846596135;
points[1180].color = giallo;

points[1181].x = 257.35325005413165;
points[1181].y = 72.33815141343155;
points[1181].color = blu;

points[1182].x = 257.15699119563396;
points[1182].y = 60.24778537231136;
points[1182].color = giallo;

points[1183].x = 256.5070939658122;
points[1183].y = 71.81311902116632;
points[1183].color = blu;

points[1184].x = 257.5063293674615;
points[1184].y = 61.18066900145205;
points[1184].color = giallo;

points[1185].x = 255.64653753775744;
points[1185].y = 71.31158886489337;
points[1185].color = blu;

points[1186].x = 257.856406474849;
points[1186].y = 62.11327724984382;
points[1186].color = giallo;

points[1187].x = 254.79980731583188;
points[1187].y = 70.78736962828563;
points[1187].color = blu;

points[1188].x = 258.169546802641;
points[1188].y = 63.06038191798465;
points[1188].color = giallo;

points[1189].x = 253.94145821127663;
points[1189].y = 70.28584858410665;
points[1189].color = blu;

points[1190].x = 258.51884599284085;
points[1190].y = 63.99327961289708;
points[1190].color = giallo;

points[1191].x = 253.08619271981198;
points[1191].y = 69.77711089023586;
points[1191].color = blu;

points[1192].x = 258.8688912354569;
points[1192].y = 64.92590012986196;
points[1192].color = giallo;

points[1193].x = 252.21923469833155;
points[1193].y = 69.286130824727;
points[1193].color = blu;

points[1194].x = 259.21966350806315;
points[1194].y = 65.85823128658618;
points[1194].color = giallo;

points[1195].x = 251.35634328063605;
points[1195].y = 68.79151316927465;
points[1195].color = blu;

points[1196].x = 259.53137109897955;
points[1196].y = 66.80588707195594;
points[1196].color = giallo;

points[1197].x = 250.5157491475103;
points[1197].y = 68.26462443980583;
points[1197].color = blu;

points[1198].x = 259.8813759960649;
points[1198].y = 67.73852300988037;
points[1198].color = giallo;

points[1199].x = 249.6518588435142;
points[1199].y = 67.76817502974495;
points[1199].color = blu;

points[1200].x = 260.23210831374115;
points[1200].y = 68.67087057304867;
points[1200].color = giallo;

points[1201].x = 248.80764545522672;
points[1201].y = 67.24018230433387;
points[1201].color = blu;

points[1202].x = 260.58356267755664;
points[1202].y = 69.60291275952312;
points[1202].color = giallo;

points[1203].x = 247.964915383609;
points[1203].y = 66.7096689382802;
points[1203].color = blu;

points[1204].x = 260.8938607566729;
points[1204].y = 70.55114588989858;
points[1204].color = giallo;

points[1205].x = 247.09971262856723;
points[1205].y = 66.21544984819094;
points[1205].color = blu;

points[1206].x = 260.57475147613417;
points[1206].y = 70.6743194932757;
points[1206].color = giallo;

points[1207].x = 246.23724860494366;
points[1207].y = 65.71673340921132;
points[1207].color = blu;

points[1208].x = 259.7055836675762;
points[1208].y = 70.18794746883073;
points[1208].color = giallo;

points[1209].x = 245.39031153592543;
points[1209].y = 65.19365910438697;
points[1209].color = blu;

points[1210].x = 258.8594464733907;
points[1210].y = 69.66340062544718;
points[1210].color = giallo;

points[1211].x = 244.5213364377816;
points[1211].y = 64.70685675905452;
points[1211].color = blu;

points[1212].x = 258.01269140191715;
points[1212].y = 69.13921599155077;
points[1212].color = giallo;

points[1213].x = 243.67156591613087;
points[1213].y = 64.18648496750751;
points[1213].color = blu;

points[1214].x = 257.152165768077;
points[1214].y = 68.63764239456485;
points[1214].color = giallo;

points[1215].x = 242.82636459752726;
points[1215].y = 63.66634958385608;
points[1215].color = blu;

points[1216].x = 256.30598840289076;
points[1216].y = 68.11264061035544;
points[1216].color = giallo;

points[1217].x = 241.96838164253222;
points[1217].y = 63.161357103624894;
points[1217].color = blu;

points[1218].x = 255.4462150779383;
points[1218].y = 67.61313460273027;
points[1218].color = giallo;

points[1219].x = 241.13848443640447;
points[1219].y = 62.61622653870682;
points[1219].color = blu;

points[1220].x = 254.58213844208888;
points[1220].y = 67.11699318475641;
points[1220].color = giallo;

points[1221].x = 240.2693389546692;
points[1221].y = 62.129944630001496;
points[1221].color = blu;

points[1222].x = 253.71548345140062;
points[1222].y = 66.62543022555772;
points[1222].color = giallo;

points[1223].x = 239.40190213322504;
points[1223].y = 61.63990725119198;
points[1223].color = blu;

points[1224].x = 252.86849182336533;
points[1224].y = 66.10428749142547;
points[1224].color = giallo;

points[1225].x = 238.560369106984;
points[1225].y = 61.10715340600173;
points[1225].color = blu;

points[1226].x = 252.0115919951826;
points[1226].y = 65.5995930673117;
points[1226].color = giallo;

points[1227].x = 237.69353753549913;
points[1227].y = 60.61592860282807;
points[1227].color = blu;

points[1228].x = 251.15463587909707;
points[1228].y = 65.09331177652405;
points[1228].color = giallo;

points[1229].x = 236.85154713590114;
points[1229].y = 60.0840555809615;
points[1229].color = blu;

points[1230].x = 250.30582021021107;
points[1230].y = 64.5718439506024;
points[1230].color = giallo;

points[1231].x = 236.0038334699971;
points[1231].y = 59.560493679273414;
points[1231].color = blu;

points[1232].x = 249.45903172958398;
points[1232].y = 64.046872148756;
points[1232].color = giallo;

points[1233].x = 235.14287602238332;
points[1233].y = 59.06065664265802;
points[1233].color = blu;

points[1234].x = 248.5906580097894;
points[1234].y = 63.558764789701776;
points[1234].color = giallo;

points[1235].x = 234.29799296077599;
points[1235].y = 58.53282594930048;
points[1235].color = blu;

points[1236].x = 247.7412277095067;
points[1236].y = 63.040166161301414;
points[1236].color = giallo;

points[1237].x = 233.43436435541474;
points[1237].y = 58.03695105921428;
points[1237].color = blu;

points[1238].x = 246.88132375258542;
points[1238].y = 62.53678473408815;
points[1238].color = giallo;

points[1239].x = 232.5694625916967;
points[1239].y = 57.542207305187304;
points[1239].color = blu;

points[1240].x = 246.01507258101304;
points[1240].y = 62.04479277452182;
points[1240].color = giallo;

points[1241].x = 231.7260100523969;
points[1241].y = 57.01295363091463;
points[1241].color = blu;

points[1242].x = 245.19013765547274;
points[1242].y = 61.49238554900012;
points[1242].color = giallo;

points[1243].x = 230.86178920074133;
points[1243].y = 56.517052032833206;
points[1243].color = blu;

points[1244].x = 244.33006918601674;
points[1244].y = 60.99017141572476;
points[1244].color = giallo;

points[1245].x = 230.0349296226246;
points[1245].y = 55.9648803601805;
points[1245].color = blu;

points[1246].x = 243.467048279855;
points[1246].y = 60.49232215682702;
points[1246].color = giallo;

points[1247].x = 229.17858116712492;
points[1247].y = 55.45941060793082;
points[1247].color = blu;

points[1248].x = 242.6297843027358;
points[1248].y = 59.958858242257705;
points[1248].color = giallo;

points[1249].x = 228.31295873852306;
points[1249].y = 54.965937236008536;
points[1249].color = blu;

points[1250].x = 241.76216307450846;
points[1250].y = 59.46963848291556;
points[1250].color = giallo;

points[1251].x = 227.47328363455816;
points[1251].y = 54.43746964794758;
points[1251].color = blu;

points[1252].x = 240.8959218463283;
points[1252].y = 58.97729671228567;
points[1252].color = giallo;

points[1253].x = 226.60795730827854;
points[1253].y = 53.943528259298596;
points[1253].color = blu;

points[1254].x = 240.05395658891965;
points[1254].y = 58.44537595038017;
points[1254].color = giallo;

points[1255].x = 225.74588785244984;
points[1255].y = 53.444090045187565;
points[1255].color = blu;

points[1256].x = 239.1879280078331;
points[1256].y = 57.952954812168485;
points[1256].color = giallo;

points[1257].x = 224.90359102690869;
points[1257].y = 52.919898920981176;
points[1257].color = blu;

points[1258].x = 238.3455845988076;
points[1258].y = 57.42141083118878;
points[1258].color = giallo;

points[1259].x = 224.0418863394959;
points[1259].y = 52.42003478308011;
points[1259].color = blu;

points[1260].x = 237.50269245143542;
points[1260].y = 56.89118562481715;
points[1260].color = giallo;

points[1261].x = 223.21754971926612;
points[1261].y = 51.85983561265379;
points[1261].color = blu;

points[1262].x = 236.63705327180992;
points[1262].y = 56.39774246768291;
points[1262].color = giallo;

points[1263].x = 222.35175539599066;
points[1263].y = 51.3666725215396;
points[1263].color = blu;

points[1264].x = 235.7946475317733;
points[1264].y = 55.86664247281532;
points[1264].color = giallo;

points[1265].x = 221.48864081360554;
points[1265].y = 50.86897141959244;
points[1265].color = blu;

points[1266].x = 234.92836306531987;
points[1266].y = 55.374381001749576;
points[1266].color = giallo;

points[1267].x = 220.64374983530556;
points[1267].y = 50.34209243693898;
points[1267].color = blu;

points[1268].x = 234.0703369487901;
points[1268].y = 54.87004227422952;
points[1268].color = giallo;

points[1269].x = 219.78144493357922;
points[1269].y = 49.84308828355825;
points[1269].color = blu;

points[1270].x = 233.2205824525435;
points[1270].y = 54.3497220965053;
points[1270].color = giallo;

points[1271].x = 218.9361037627946;
points[1271].y = 49.31685605743379;
points[1271].color = blu;

points[1272].x = 232.3640379252098;
points[1272].y = 53.842937939678784;
points[1272].color = giallo;

points[1273].x = 218.07444997618364;
points[1273].y = 48.81691624472414;
points[1273].color = blu;

points[1274].x = 231.53833662301471;
points[1274].y = 53.29004909280659;
points[1274].color = giallo;

points[1275].x = 217.22854879789895;
points[1275].y = 48.29151450204079;
points[1275].color = blu;

points[1276].x = 230.67276000784287;
points[1276].y = 52.79649336590468;
points[1276].color = giallo;

points[1277].x = 216.3842470816842;
points[1277].y = 47.76366417198895;
points[1277].color = blu;

points[1278].x = 229.80475432031574;
points[1278].y = 52.30761349553442;
points[1278].color = giallo;

points[1279].x = 215.51576790968957;
points[1279].y = 47.27611626434484;
points[1279].color = blu;

points[1280].x = 228.96736325520766;
points[1280].y = 51.774743642595794;
points[1280].color = giallo;

points[1281].x = 214.67614234962784;
points[1281].y = 46.73928480740302;
points[1281].color = blu;

points[1282].x = 228.09887663904618;
points[1282].y = 51.286876889954506;
points[1282].color = giallo;

points[1283].x = 213.80671332408747;
points[1283].y = 46.253506192215;
points[1283].color = blu;

points[1284].x = 227.26149105113288;
points[1284].y = 50.75375888930915;
points[1284].color = giallo;

points[1285].x = 212.95465901752647;
points[1285].y = 45.73677272934243;
points[1285].color = blu;

points[1286].x = 226.40530665067047;
points[1286].y = 50.24654064811737;
points[1286].color = giallo;

points[1287].x = 212.10701361287312;
points[1287].y = 45.213797706369434;
points[1287].color = blu;

points[1288].x = 225.5458752318662;
points[1288].y = 49.743453637546736;
points[1288].color = giallo;

points[1289].x = 211.24848004383793;
points[1289].y = 44.70951730230634;
points[1289].color = blu;

points[1290].x = 224.71171203998858;
points[1290].y = 49.19695021116885;
points[1290].color = giallo;

points[1291].x = 210.4002245158484;
points[1291].y = 44.18733933860267;
points[1291].color = blu;

points[1292].x = 223.84386107021226;
points[1292].y = 48.7077511062846;
points[1292].color = giallo;

points[1293].x = 209.54818557867054;
points[1293].y = 43.673249953433434;
points[1293].color = blu;

points[1294].x = 222.99226866025495;
points[1294].y = 48.19290221649709;
points[1294].color = giallo;

points[1295].x = 208.6804410952512;
points[1295].y = 43.183833378067725;
points[1295].color = blu;

points[1296].x = 222.1346485418062;
points[1296].y = 47.685490430087896;
points[1296].color = giallo;

points[1297].x = 207.83913864079432;
points[1297].y = 42.65062331883859;
points[1297].color = blu;

points[1298].x = 221.2854456438245;
points[1298].y = 47.16649038048745;
points[1298].color = giallo;

points[1299].x = 206.97199631079522;
points[1299].y = 42.16000250574794;
points[1299].color = blu;

points[1300].x = 220.43790863407918;
points[1300].y = 46.643371062825466;
points[1300].color = giallo;

points[1301].x = 206.13124197272825;
points[1301].y = 41.62618713629705;
points[1301].color = blu;

points[1302].x = 219.57842311207256;
points[1302].y = 46.14035747167843;
points[1302].color = giallo;

points[1303].x = 205.28394035616543;
points[1303].y = 41.108643539118816;
points[1303].color = blu;

points[1304].x = 218.73148197636456;
points[1304].y = 45.616430386520726;
points[1304].color = giallo;

points[1305].x = 204.42864547472553;
points[1305].y = 40.60037633490715;
points[1305].color = blu;

points[1306].x = 217.87627659605684;
points[1306].y = 45.104889551228496;
points[1306].color = giallo;

points[1307].x = 203.58966166311384;
points[1307].y = 40.07447476887137;
points[1307].color = blu;

points[1308].x = 217.00800397657554;
points[1308].y = 44.61694353103322;
points[1308].color = giallo;

points[1309].x = 202.7216986330224;
points[1309].y = 39.5855068137319;
points[1309].color = blu;

points[1310].x = 216.1670551217167;
points[1310].y = 44.0826491357195;
points[1310].color = giallo;

points[1311].x = 201.85840500884277;
points[1311].y = 39.0932074864934;
points[1311].color = blu;

points[1312].x = 215.29989682290244;
points[1312].y = 43.59250737672952;
points[1312].color = giallo;

points[1313].x = 201.01871622406108;
points[1313].y = 38.564764140505105;
points[1313].color = blu;

points[1314].x = 214.45621161229195;
points[1314].y = 43.06364593986648;
points[1314].color = giallo;

points[1315].x = 200.1760497955566;
points[1315].y = 38.033684842519506;
points[1315].color = blu;

points[1316].x = 213.61097099227806;
points[1316].y = 42.537261305809515;
points[1316].color = giallo;

points[1317].x = 199.32632714787607;
points[1317].y = 37.51332657750407;
points[1317].color = blu;

points[1318].x = 212.7484431210763;
points[1318].y = 42.038643238242386;
points[1318].color = giallo;

points[1319].x = 198.4684142712705;
points[1319].y = 37.008810232549806;
points[1319].color = blu;

points[1320].x = 211.9037448614852;
points[1320].y = 41.51142251884138;
points[1320].color = giallo;

points[1321].x = 197.6202367895463;
points[1321].y = 36.485961266978734;
points[1321].color = blu;

points[1322].x = 211.0404930268581;
points[1322].y = 41.01393964021531;
points[1322].color = giallo;

points[1323].x = 196.75971844963303;
points[1323].y = 35.9854598372954;
points[1323].color = blu;

points[1324].x = 210.17458220186012;
points[1324].y = 40.52098886503411;
points[1324].color = giallo;

points[1325].x = 195.89404918042223;
points[1325].y = 35.49207095448295;
points[1325].color = blu;

points[1326].x = 209.33237673558884;
points[1326].y = 39.989519275379465;
points[1326].color = giallo;

points[1327].x = 195.05117844624988;
points[1327].y = 34.96180794852691;
points[1327].color = blu;

points[1328].x = 208.46582807299387;
points[1328].y = 39.49775422379361;
points[1328].color = giallo;

points[1329].x = 194.1861658146136;
points[1329].y = 34.46725624365248;
points[1329].color = blu;

points[1330].x = 207.6465357074662;
points[1330].y = 38.936248562123374;
points[1330].color = giallo;

points[1331].x = 193.3428005040414;
points[1331].y = 33.93785372256095;
points[1331].color = blu;

points[1332].x = 206.7840755998459;
points[1332].y = 38.43752610381392;
points[1332].color = giallo;

points[1333].x = 192.50085549026096;
points[1333].y = 33.40589452201316;
points[1333].color = blu;

points[1334].x = 205.94236681654053;
points[1334].y = 37.9124339156536;
points[1334].color = giallo;

points[1335].x = 191.63458484079055;
points[1335].y = 32.91360739086372;
points[1335].color = blu;

points[1336].x = 205.08154284860697;
points[1336].y = 37.415986036323886;
points[1336].color = giallo;

points[1337].x = 190.79216792078574;
points[1337].y = 32.38252785700837;
points[1337].color = blu;

points[1338].x = 204.21587296573645;
points[1338].y = 36.92259826040488;
points[1338].color = giallo;

points[1339].x = 189.92654287199142;
points[1339].y = 31.889059211278045;
points[1339].color = blu;

points[1340].x = 203.37402677480077;
points[1340].y = 36.398728277231655;
points[1340].color = giallo;

points[1341].x = 189.06360701778777;
points[1341].y = 31.39107631845847;
points[1341].color = blu;

points[1342].x = 202.51278898169622;
points[1342].y = 35.90205136387327;
points[1342].color = giallo;

points[1343].x = 188.21866921612224;
points[1343].y = 30.86422842214726;
points[1343].color = blu;

points[1344].x = 201.67074401449807;
points[1344].y = 35.370281461042936;
points[1344].color = giallo;

points[1345].x = 187.35646447203115;
points[1345].y = 30.365115572623438;
points[1345].color = blu;

points[1346].x = 200.82726936985912;
points[1346].y = 34.84106518580346;
points[1346].color = giallo;

points[1347].x = 186.49727705296118;
points[1347].y = 29.86170020790006;
points[1347].color = blu;

points[1348].x = 199.96239868964108;
points[1348].y = 34.34626772050055;
points[1348].color = giallo;

points[1349].x = 185.65162776938934;
points[1349].y = 29.34040646808538;
points[1349].color = blu;

points[1350].x = 199.11942706491385;
points[1350].y = 33.816182811280996;
points[1350].color = giallo;

points[1351].x = 184.80181075815403;
points[1351].y = 28.822487464066374;
points[1351].color = blu;

points[1352].x = 198.25389284064272;
points[1352].y = 33.32255117160952;
points[1352].color = giallo;

points[1353].x = 183.93352812781504;
points[1353].y = 28.334186901729097;
points[1353].color = blu;

points[1354].x = 197.39281890293364;
points[1354].y = 32.822889637959264;
points[1354].color = giallo;

points[1355].x = 183.0926362869318;
points[1355].y = 27.800141148408937;
points[1355].color = blu;

points[1356].x = 196.54523020068658;
points[1356].y = 32.29913565333658;
points[1356].color = giallo;

points[1357].x = 182.22493687747496;
points[1357].y = 27.310632935960818;
points[1357].color = blu;

points[1358].x = 195.68677017394367;
points[1358].y = 31.795473174686034;
points[1358].color = giallo;

points[1359].x = 181.38147505315376;
points[1359].y = 26.780711829057402;
points[1359].color = blu;

points[1360].x = 194.83691107664455;
points[1360].y = 31.275277358978325;
points[1360].color = giallo;

points[1361].x = 180.53192492103852;
points[1361].y = 26.26014611047787;
points[1361].color = blu;

points[1362].x = 193.99486091182942;
points[1362].y = 30.743311528522092;
points[1362].color = giallo;

points[1363].x = 179.6747150731281;
points[1363].y = 25.754522716346308;
points[1363].color = blu;

points[1364].x = 193.12745182177432;
points[1364].y = 30.253218633452068;
points[1364].color = giallo;

points[1365].x = 178.8082319716755;
points[1365].y = 25.262634098105615;
points[1365].color = blu;

points[1366].x = 192.28634651259318;
points[1366].y = 29.719611094729323;
points[1366].color = giallo;

points[1367].x = 177.96924322609777;
points[1367].y = 24.732921504400924;
points[1367].color = blu;

points[1368].x = 191.4183431804007;
points[1368].y = 29.230726360749546;
points[1368].color = giallo;

points[1369].x = 177.10329162227143;
points[1369].y = 24.240045360574385;
points[1369].color = blu;

points[1370].x = 190.56737063122426;
points[1370].y = 28.714814719230457;
points[1370].color = giallo;

points[1371].x = 176.23516418996962;
points[1371].y = 23.751817394828883;
points[1371].color = blu;

points[1372].x = 189.71938351284177;
points[1372].y = 28.192289311585476;
points[1372].color = giallo;

points[1373].x = 175.379627655984;
points[1373].y = 23.240836823618842;
points[1373].color = blu;

points[1374].x = 188.8604919613208;
points[1374].y = 27.688479764300453;
points[1374].color = giallo;

points[1375].x = 174.52606382196063;
points[1375].y = 22.72930697388248;
points[1375].color = blu;

points[1376].x = 188.01600096686323;
points[1376].y = 27.167407480042613;
points[1376].color = giallo;

points[1377].x = 173.67316872034957;
points[1377].y = 22.213938035046997;
points[1377].color = blu;

points[1378].x = 187.15792025809003;
points[1378].y = 26.665519927119842;
points[1378].color = giallo;

points[1379].x = 172.81682258324415;
points[1379].y = 21.706913851009865;
points[1379].color = blu;

points[1380].x = 186.2954304889557;
points[1380].y = 26.166843061831457;
points[1380].color = giallo;

points[1381].x = 171.96692481269835;
points[1381].y = 21.18676399686287;
points[1381].color = blu;

points[1382].x = 185.43020173881254;
points[1382].y = 25.672669533012872;
points[1382].color = giallo;

points[1383].x = 171.1083028517802;
points[1383].y = 20.683353802011716;
points[1383].color = blu;

points[1384].x = 184.58748957089236;
points[1384].y = 25.14212410537821;
points[1384].color = giallo;

points[1385].x = 170.24216826506796;
points[1385].y = 20.19081398315185;
points[1385].color = blu;

points[1386].x = 183.72161388931318;
points[1386].y = 24.649109012484377;
points[1386].color = giallo;

points[1387].x = 169.37866890778275;
points[1387].y = 19.693728202230954;
points[1387].color = blu;

points[1388].x = 182.87938062298284;
points[1388].y = 24.1176910514557;
points[1388].color = giallo;

points[1389].x = 168.50623199020853;
points[1389].y = 19.21333823800969;
points[1389].color = blu;

points[1390].x = 182.03570185745554;
points[1390].y = 23.588818861621036;
points[1390].color = giallo;

points[1391].x = 167.59692458275387;
points[1391].y = 18.811629633175748;
points[1391].color = blu;

points[1392].x = 181.17110107584256;
points[1392].y = 23.093558515934877;
points[1392].color = giallo;

points[1393].x = 166.68500036833115;
points[1393].y = 18.414801107345507;
points[1393].color = blu;

points[1394].x = 180.30397379563902;
points[1394].y = 22.602908177184595;
points[1394].color = giallo;

points[1395].x = 165.74943206756504;
points[1395].y = 18.086098357718573;
points[1395].color = blu;

points[1396].x = 179.4658983585321;
points[1396].y = 22.071439751160455;
points[1396].color = giallo;

points[1397].x = 164.81590022752775;
points[1397].y = 17.738750542713497;
points[1397].color = blu;

points[1398].x = 178.59826883507108;
points[1398].y = 21.58178945782511;
points[1398].color = giallo;

points[1399].x = 163.84462606926058;
points[1399].y = 17.52570615058037;
points[1399].color = blu;

points[1400].x = 177.72949652689263;
points[1400].y = 21.09480232385373;
points[1400].color = giallo;

points[1401].x = 162.89513932121886;
points[1401].y = 17.24640051527595;
points[1401].color = blu;

points[1402].x = 176.88506438212724;
points[1402].y = 20.56716078645939;
points[1402].color = giallo;

points[1403].x = 161.91139302351283;
points[1403].y = 17.11643992318256;
points[1403].color = blu;

points[1404].x = 176.02146363150885;
points[1404].y = 20.07023868528529;
points[1404].color = giallo;

points[1405].x = 160.94131726290024;
points[1405].y = 16.903377110694183;
points[1405].color = blu;

points[1406].x = 175.1775524990931;
points[1406].y = 19.54175240750575;
points[1406].color = giallo;

points[1407].x = 159.9560220625473;
points[1407].y = 16.79080675422139;
points[1407].color = blu;

points[1408].x = 174.31326122098292;
points[1408].y = 19.04596895925698;
points[1408].color = giallo;

points[1409].x = 158.9560220625473;
points[1409].y = 16.79080675422139;
points[1409].color = blu;

points[1410].x = 173.46986412194434;
points[1410].y = 18.516620962352068;
points[1410].color = giallo;

points[1411].x = 157.96190725881252;
points[1411].y = 16.729456535605618;
points[1411].color = blu;

points[1412].x = 172.60489187371874;
points[1412].y = 18.02199900325772;
points[1412].color = giallo;

points[1413].x = 156.97012958112796;
points[1413].y = 16.79080675422139;
points[1413].color = blu;

points[1414].x = 171.7374329237181;
points[1414].y = 17.53200592423604;
points[1414].color = giallo;

points[1415].x = 155.97012958112796;
points[1415].y = 16.79080675422139;
points[1415].color = blu;

points[1416].x = 170.88422244657914;
points[1416].y = 17.019887097574895;
points[1416].color = giallo;

points[1417].x = 154.98483438077503;
points[1417].y = 16.903377110694183;
points[1417].color = blu;

points[1418].x = 170.0029350163156;
points[1418].y = 16.551764408037595;
points[1418].color = giallo;

points[1419].x = 154.01846522996516;
points[1419].y = 17.12244329987119;
points[1419].color = blu;

points[1420].x = 169.11646146423467;
points[1420].y = 16.108976366107605;
points[1420].color = giallo;

points[1421].x = 153.0432423380829;
points[1421].y = 17.29993244802857;
points[1421].color = blu;

points[1422].x = 168.2047222815022;
points[1422].y = 15.711762754890092;
points[1422].color = giallo;

points[1423].x = 152.0887146380785;
points[1423].y = 17.560853055907014;
points[1423].color = blu;

points[1424].x = 167.28634645318752;
points[1424].y = 15.325565206579412;
points[1424].color = giallo;

points[1425].x = 151.13331018871713;
points[1425].y = 17.834229918180988;
points[1425].color = blu;

points[1426].x = 166.32806961952372;
points[1426].y = 15.052694989021127;
points[1426].color = giallo;

points[1427].x = 150.2026661605388;
points[1427].y = 18.18852259275263;
points[1427].color = blu;

points[1428].x = 165.39246387420627;
points[1428].y = 14.7126538398873;
points[1428].color = giallo;

points[1429].x = 149.28918121733557;
points[1429].y = 18.586254159257564;
points[1429].color = blu;

points[1430].x = 164.43882699693557;
points[1430].y = 14.452848920111961;
points[1430].color = giallo;

points[1431].x = 148.38406060582898;
points[1431].y = 18.999829262766198;
points[1431].color = blu;

points[1432].x = 163.46553334600594;
points[1432].y = 14.246390954486515;
points[1432].color = giallo;

points[1433].x = 147.51903199520027;
points[1433].y = 19.489030065221378;
points[1433].color = blu;

points[1434].x = 162.4855304897154;
points[1434].y = 14.100602660209518;
points[1434].color = giallo;

points[1435].x = 146.6714388153661;
points[1435].y = 20.00623315596254;
points[1435].color = blu;

points[1436].x = 161.51082273705129;
points[1436].y = 13.903377110694183;
points[1436].color = giallo;

points[1437].x = 145.85082404109764;
points[1437].y = 20.572170921592637;
points[1437].color = blu;

points[1438].x = 160.525527535761;
points[1438].y = 13.790807365378292;
points[1438].color = giallo;

points[1439].x = 145.05867138409755;
points[1439].y = 21.171466970719116;
points[1439].color = blu;

points[1440].x = 159.52552753669846;
points[1440].y = 13.79080675422139;
points[1440].color = giallo;

points[1441].x = 144.30144115539323;
points[1441].y = 21.816126842950666;
points[1441].color = blu;

points[1442].x = 158.53883559630407;
points[1442].y = 13.701816676977774;
points[1442].color = giallo;

points[1443].x = 143.5887550615508;
points[1443].y = 22.51148889822257;
points[1443].color = blu;

points[1444].x = 157.54024714398707;
points[1444].y = 13.679374174274084;
points[1444].color = giallo;

points[1445].x = 142.8911051332588;
points[1445].y = 23.226462865085075;
points[1445].color = blu;

points[1446].x = 156.55493713599276;
points[1446].y = 13.79080675422139;
points[1446].color = giallo;

points[1447].x = 142.25946363075866;
points[1447].y = 23.993191366930184;
points[1447].color = blu;

points[1448].x = 155.55493713599276;
points[1448].y = 13.79080675422139;
points[1448].color = giallo;

points[1449].x = 141.66422247477985;
points[1449].y = 24.791036977132947;
points[1449].color = blu;

points[1450].x = 154.56964193563994;
points[1450].y = 13.903377110694183;
points[1450].color = giallo;

points[1451].x = 141.08231401098672;
points[1451].y = 25.598105769721148;
points[1451].color = blu;

points[1452].x = 153.59374516441758;
points[1452].y = 14.070964812281005;
points[1452].color = giallo;

points[1453].x = 140.53262633834154;
points[1453].y = 26.42422855607228;
points[1453].color = blu;

points[1454].x = 152.61836691393722;
points[1454].y = 14.247603367928317;
points[1454].color = giallo;

points[1455].x = 139.92106992095322;
points[1455].y = 27.209631285645813;
points[1455].color = blu;

points[1456].x = 151.64594731132524;
points[1456].y = 14.457807398942814;
points[1456].color = giallo;

points[1457].x = 139.3454845436482;
points[1457].y = 28.021267508583218;
points[1457].color = blu;

points[1458].x = 150.69520130220863;
points[1458].y = 14.738801380710644;
points[1458].color = giallo;

points[1459].x = 138.8003251431681;
points[1459].y = 28.85623727592327;
points[1459].color = blu;

points[1460].x = 149.76147667934387;
points[1460].y = 15.085463612439018;
points[1460].color = giallo;

points[1461].x = 138.2244458189879;
points[1461].y = 29.663039727180557;
points[1461].color = blu;

points[1462].x = 148.84018630127036;
points[1462].y = 15.464452013170945;
points[1462].color = giallo;

points[1463].x = 137.64586810864358;
points[1463].y = 30.4729351982512;
points[1463].color = blu;

points[1464].x = 147.90903150506196;
points[1464].y = 15.81755164502766;
points[1464].color = giallo;

points[1465].x = 137.06689900095557;
points[1465].y = 31.282544249559656;
points[1465].color = blu;

points[1466].x = 147.02270490879627;
points[1466].y = 16.262047970479387;
points[1466].color = giallo;

points[1467].x = 136.4875441747635;
points[1467].y = 32.09186024646392;
points[1467].color = blu;

points[1468].x = 146.1532438373635;
points[1468].y = 16.747753077111735;
points[1468].color = giallo;

points[1469].x = 135.91356533560491;
points[1469].y = 32.90569121808605;
points[1469].color = blu;

points[1470].x = 145.2860199495828;
points[1470].y = 17.238213092996737;
points[1470].color = giallo;

points[1471].x = 135.33164704884192;
points[1471].y = 33.712750217794635;
points[1471].color = blu;

points[1472].x = 144.47446732249944;
points[1472].y = 17.81417824880931;
points[1472].color = giallo;

points[1473].x = 134.7825371721294;
points[1473].y = 34.543965855928334;
points[1473].color = blu;

points[1474].x = 143.68293773942534;
points[1474].y = 18.41713028474487;
points[1474].color = giallo;

points[1475].x = 134.2052498197574;
points[1475].y = 35.35473231499217;
points[1475].color = blu;

points[1476].x = 142.8779286446089;
points[1476].y = 19.001795209347947;
points[1476].color = giallo;

points[1477].x = 133.64708615162823;
points[1477].y = 36.17669289954744;
points[1477].color = blu;

points[1478].x = 142.16847728201657;
points[1478].y = 19.706450029208025;
points[1478].color = giallo;

points[1479].x = 133.0667760350173;
points[1479].y = 36.98523404156284;
points[1479].color = blu;

points[1480].x = 141.41124705331225;
points[1480].y = 20.351109901439543;
points[1480].color = giallo;

points[1481].x = 132.48864572391412;
points[1481].y = 37.79516937613326;
points[1481].color = blu;

points[1482].x = 140.75023978906427;
points[1482].y = 21.09561057471395;
points[1482].color = giallo;

points[1483].x = 131.9478596013443;
points[1483].y = 38.62845579191326;
points[1483].color = blu;

points[1484].x = 140.09637375956606;
points[1484].y = 21.84581637951183;
points[1484].color = giallo;

points[1485].x = 131.37000072297988;
points[1485].y = 39.438849784489086;
points[1485].color = blu;

points[1486].x = 139.4795462189948;
points[1486].y = 22.624157468502432;
points[1486].color = giallo;

points[1487].x = 130.79174183541596;
points[1487].y = 40.248970553355875;
points[1487].color = blu;

points[1488].x = 138.89895689686747;
points[1488].y = 23.4324580569831;
points[1488].color = giallo;

points[1489].x = 130.23233689234843;
points[1489].y = 41.06951282998664;
points[1489].color = blu;

points[1490].x = 138.31356160459177;
points[1490].y = 24.236787845469625;
points[1490].color = giallo;

points[1491].x = 129.65712763734572;
points[1491].y = 41.88147064304449;
points[1491].color = blu;

points[1492].x = 137.74117257525413;
points[1492].y = 25.047132060126316;
points[1492].color = giallo;

points[1493].x = 129.08148949603643;
points[1493].y = 42.69320243458159;
points[1493].color = blu;

points[1494].x = 137.16165300827623;
points[1494].y = 25.85631955918054;
points[1494].color = giallo;

points[1495].x = 128.53181393432823;
points[1495].y = 43.525399213745324;
points[1495].color = blu;

points[1496].x = 136.5825169103372;
points[1496].y = 26.665803049802612;
points[1496].color = giallo;

points[1497].x = 127.97427704387512;
points[1497].y = 44.34707921467837;
points[1497].color = blu;

points[1498].x = 135.990583186154;
points[1498].y = 27.46387578211059;
points[1498].color = giallo;

points[1499].x = 127.43081386005554;
points[1499].y = 45.17879999119928;
points[1499].color = blu;

points[1500].x = 135.4277362403536;
points[1500].y = 28.282270435967963;
points[1500].color = giallo;

points[1501].x = 126.85257578745703;
points[1501].y = 45.988935229370746;
points[1501].color = blu;

points[1502].x = 134.88132896990842;
points[1502].y = 29.116491663218124;
points[1502].color = giallo;

points[1503].x = 126.29133891122066;
points[1503].y = 46.812642584061535;
points[1503].color = blu;

points[1504].x = 134.30618258013487;
points[1504].y = 29.928481706581977;
points[1504].color = giallo;

points[1505].x = 125.74712170537971;
points[1505].y = 47.64816887767455;
points[1505].color = blu;

points[1506].x = 133.72949602830764;
points[1506].y = 30.739381233841225;
points[1506].color = giallo;

points[1507].x = 125.1838500142933;
points[1507].y = 48.47086796870836;
points[1507].color = blu;

points[1508].x = 133.14909210007704;
points[1508].y = 31.547842420850834;
points[1508].color = giallo;

points[1509].x = 124.64096616580841;
points[1509].y = 49.30693028046696;
points[1509].color = blu;

points[1510].x = 132.569058817871;
points[1510].y = 32.356615665296495;
points[1510].color = giallo;

points[1511].x = 124.08122581383576;
points[1511].y = 50.13227278365721;
points[1511].color = blu;

points[1512].x = 131.9894016466348;
points[1512].y = 33.16569419669818;
points[1512].color = giallo;

points[1513].x = 123.5340981727081;
points[1513].y = 50.96523619515028;
points[1513].color = blu;

points[1514].x = 131.41012600647386;
points[1514].y = 33.975071212906606;
points[1514].color = giallo;

points[1515].x = 122.97795083892336;
points[1515].y = 51.793137489459724;
points[1515].color = blu;

points[1516].x = 130.85931883892619;
points[1516].y = 34.80217287779383;
points[1516].color = giallo;

points[1517].x = 122.45502304127618;
points[1517].y = 52.6408183729734;
points[1517].color = blu;

points[1518].x = 130.30117686797288;
points[1518].y = 35.62634700667371;
points[1518].color = giallo;

points[1519].x = 121.88956570786188;
points[1519].y = 53.457118634325646;
points[1519].color = blu;

points[1520].x = 129.7195720238328;
points[1520].y = 36.43371383781928;
points[1520].color = giallo;

points[1521].x = 121.32339157853578;
points[1521].y = 54.27086243477301;
points[1521].color = blu;

points[1522].x = 129.1521502227927;
points[1522].y = 37.248385400661874;
points[1522].color = giallo;

points[1523].x = 120.79200836613063;
points[1523].y = 55.10898076481284;
points[1523].color = blu;

points[1524].x = 128.60064348100227;
points[1524].y = 38.07940608584102;
points[1524].color = giallo;

points[1525].x = 120.25184627889784;
points[1525].y = 55.946818287920145;
points[1525].color = blu;

points[1526].x = 128.02472455704662;
points[1526].y = 38.89098447522771;
points[1526].color = giallo;

points[1527].x = 119.75162931834777;
points[1527].y = 56.80828451182672;
points[1527].color = blu;

points[1528].x = 127.46064483147421;
points[1528].y = 39.70622764668374;
points[1528].color = giallo;

points[1529].x = 119.19402518709207;
points[1529].y = 57.629923080193365;
points[1529].color = blu;

points[1530].x = 126.87953614593702;
points[1530].y = 40.51406266679791;
points[1530].color = giallo;

points[1531].x = 118.66702973354346;
points[1531].y = 58.47536679199803;
points[1531].color = blu;

points[1532].x = 126.31356742682168;
points[1532].y = 41.334652823001406;
points[1532].color = giallo;

points[1533].x = 118.12522521517123;
points[1533].y = 59.307359676671226;
points[1533].color = blu;

points[1534].x = 125.76641338327029;
points[1534].y = 42.16841941775123;
points[1534].color = giallo;

points[1535].x = 117.62463319558567;
points[1535].y = 60.16880327822499;
points[1535].color = blu;

points[1536].x = 125.19928754913731;
points[1536].y = 42.98148857453794;
points[1536].color = giallo;

points[1537].x = 117.09869346090963;
points[1537].y = 61.01433961504206;
points[1537].color = blu;

points[1538].x = 124.63426163240244;
points[1538].y = 43.79814068095409;
points[1538].color = giallo;

points[1539].x = 116.6160896367508;
points[1539].y = 61.885501797685514;
points[1539].color = blu;

points[1540].x = 124.0854213184408;
points[1540].y = 44.630863541504596;
points[1540].color = giallo;

points[1541].x = 116.30991934485664;
points[1541].y = 62.83077059018271;
points[1541].color = blu;

points[1542].x = 123.52965104917857;
points[1542].y = 45.4579750770485;
points[1542].color = giallo;

points[1543].x = 116.00032591684918;
points[1543].y = 63.774551226224084;
points[1543].color = blu;

points[1544].x = 122.9771007007331;
points[1544].y = 46.28831187311178;
points[1544].color = giallo;

points[1545].x = 115.82252992628386;
points[1545].y = 64.75154070017521;
points[1545].color = blu;

points[1546].x = 122.42571607043475;
points[1546].y = 47.11831802012891;
points[1546].color = giallo;

points[1547].x = 115.64634146341463;
points[1547].y = 65.7263161379799;
points[1547].color = blu;

points[1548].x = 121.86951951228353;
points[1548].y = 47.94618509663955;
points[1548].color = giallo;

points[1549].x = 115.61049366011363;
points[1549].y = 66.72369416584692;
points[1549].color = blu;

points[1550].x = 121.32244775342934;
points[1550].y = 48.77918812190982;
points[1550].color = giallo;

points[1551].x = 115.53377110694184;
points[1551].y = 67.71161133833408;
points[1551].color = blu;

points[1552].x = 120.76266122875275;
points[1552].y = 49.60449703536227;
points[1552].color = giallo;

points[1553].x = 115.53377110694184;
points[1553].y = 68.71161133833408;
points[1553].color = blu;

points[1554].x = 120.21983227354023;
points[1554].y = 50.440600320386075;
points[1554].color = giallo;

points[1555].x = 115.53377110694184;
points[1555].y = 69.71161133833408;
points[1555].color = blu;

points[1556].x = 119.65856209481348;
points[1556].y = 51.26022293214587;
points[1556].color = giallo;

points[1557].x = 115.53377110694184;
points[1557].y = 70.71161133833408;
points[1557].color = blu;

points[1558].x = 119.11557728463305;
points[1558].y = 52.092228443564345;
points[1558].color = giallo;

points[1559].x = 115.53358685922476;
points[1559].y = 71.71161035830632;
points[1559].color = blu;

points[1560].x = 118.58607604892204;
points[1560].y = 52.931331314145744;
points[1560].color = giallo;

points[1561].x = 115.42120075046904;
points[1561].y = 72.69690653868811;
points[1561].color = blu;

points[1562].x = 118.0367736430017;
points[1562].y = 53.758332644711345;
points[1562].color = giallo;

points[1563].x = 115.42120075046904;
points[1563].y = 73.69690653868811;
points[1563].color = blu;

points[1564].x = 117.48036075812519;
points[1564].y = 54.58604892991615;
points[1564].color = giallo;

points[1565].x = 115.30863039399624;
points[1565].y = 74.68220173904164;
points[1565].color = blu;

points[1566].x = 116.95524659801266;
points[1566].y = 55.42774361001848;
points[1566].color = giallo;

points[1567].x = 115.18840378568365;
points[1567].y = 75.66723877015428;
points[1567].color = blu;

points[1568].x = 116.45749377067129;
points[1568].y = 56.29082558079208;
points[1568].color = giallo;

points[1569].x = 114.97499589703283;
points[1569].y = 76.6388803544309;
points[1569].color = blu;

points[1570].x = 115.9227869599359;
points[1570].y = 57.13156508051888;
points[1570].color = giallo;

points[1571].x = 114.62409923436032;
points[1571].y = 77.57116013942752;
points[1571].color = blu;

points[1572].x = 115.38931497178386;
points[1572].y = 57.96867394007515;
points[1572].color = giallo;

points[1573].x = 114.14195056763307;
points[1573].y = 78.42965120181773;
points[1573].color = blu;

points[1574].x = 114.8569994025611;
points[1574].y = 58.81043251976163;
points[1574].color = giallo;

points[1575].x = 113.56291944087724;
points[1575].y = 79.23382304151916;
points[1575].color = blu;

points[1576].x = 114.32724908930643;
points[1576].y = 59.653595846976216;
points[1576].color = giallo;

points[1577].x = 112.85581265969067;
points[1577].y = 79.94092982270573;
points[1577].color = blu;

points[1578].x = 113.8804991709427;
points[1578].y = 60.54695898417844;
points[1578].color = giallo;

points[1579].x = 112.06180615077899;
points[1579].y = 80.54129964527287;
points[1579].color = blu;

points[1580].x = 113.4968946329153;
points[1580].y = 61.4662700697078;
points[1580].color = giallo;

points[1581].x = 111.14926925135987;
points[1581].y = 80.94097429199299;
points[1581].color = blu;

points[1582].x = 113.18235359315635;
points[1582].y = 62.408244841044976;
points[1582].color = giallo;

points[1583].x = 110.16702267752041;
points[1583].y = 81.06848030018762;
points[1583].color = blu;

points[1584].x = 112.98405253283302;
points[1584].y = 63.38276767391703;
points[1584].color = giallo;

points[1585].x = 109.16702267752041;
points[1585].y = 81.06848030018762;
points[1585].color = blu;

points[1586].x = 112.76469760139396;
points[1586].y = 64.35250139016176;
points[1586].color = giallo;

points[1587].x = 108.16702267752041;
points[1587].y = 81.06848030018762;
points[1587].color = blu;

points[1588].x = 112.64634146341463;
points[1588].y = 65.33762264850074;
points[1588].color = giallo;

points[1589].x = 107.16755775775579;
points[1589].y = 81.07120561812178;
points[1589].color = blu;

points[1590].x = 112.54546877910863;
points[1590].y = 66.3234055013044;
points[1590].color = giallo;

points[1591].x = 106.18172747716733;
points[1591].y = 81.18105065666042;
points[1591].color = blu;

points[1592].x = 112.53377110694184;
points[1592].y = 67.32291784885484;
points[1592].color = giallo;

points[1593].x = 105.18172747716733;
points[1593].y = 81.18105065666042;
points[1593].color = blu;

points[1594].x = 112.53377110694184;
points[1594].y = 68.32291784885484;
points[1594].color = giallo;

points[1595].x = 104.18172747716733;
points[1595].y = 81.18105065666042;
points[1595].color = blu;

points[1596].x = 112.53377110694184;
points[1596].y = 69.32291784885484;
points[1596].color = giallo;

points[1597].x = 103.18172747716733;
points[1597].y = 81.18105065666042;
points[1597].color = blu;

points[1598].x = 112.53377110694184;
points[1598].y = 70.32291784885484;
points[1598].color = giallo;

points[1599].x = 102.18172747716733;
points[1599].y = 81.18105065666042;
points[1599].color = blu;

points[1600].x = 112.50146907052645;
points[1600].y = 71.31703786568247;
points[1600].color = giallo;

points[1601].x = 101.19643227681364;
points[1601].y = 81.2936210131332;
points[1601].color = blu;

points[1602].x = 112.42120075046904;
points[1602].y = 72.30821304920883;
points[1602].color = giallo;

points[1603].x = 100.19643227681364;
points[1603].y = 81.2936210131332;
points[1603].color = blu;

points[1604].x = 112.40264274072425;
points[1604].y = 73.30470849136461;
points[1604].color = giallo;

points[1605].x = 99.19643227681364;
points[1605].y = 81.2936210131332;
points[1605].color = blu;

points[1606].x = 112.30001490033226;
points[1606].y = 74.29184014881481;
points[1606].color = giallo;

points[1607].x = 98.19643227681364;
points[1607].y = 81.2936210131332;
points[1607].color = blu;

points[1608].x = 112.12754623680382;
points[1608].y = 75.26767400726207;
points[1608].color = giallo;

points[1609].x = 97.19643227681364;
points[1609].y = 81.2936210131332;
points[1609].color = blu;

points[1610].x = 111.84578208634795;
points[1610].y = 76.2157061425464;
points[1610].color = giallo;

points[1611].x = 96.19643227681364;
points[1611].y = 81.2936210131332;
points[1611].color = blu;

points[1612].x = 111.40226083057601;
points[1612].y = 77.09897746138148;
points[1612].color = giallo;

points[1613].x = 95.19643227681364;
points[1613].y = 81.2936210131332;
points[1613].color = blu;

points[1614].x = 110.67727300758987;
points[1614].y = 77.76838422068437;
points[1614].color = giallo;

points[1615].x = 94.19643227681364;
points[1615].y = 81.2936210131332;
points[1615].color = blu;

points[1616].x = 109.76041658930926;
points[1616].y = 78.06848030018762;
points[1616].color = giallo;

points[1617].x = 93.19643227681364;
points[1617].y = 81.2936210131332;
points[1617].color = blu;

points[1618].x = 108.76041658930926;
points[1618].y = 78.06848030018762;
points[1618].color = giallo;

points[1619].x = 92.19643227681364;
points[1619].y = 81.2936210131332;
points[1619].color = blu;

points[1620].x = 107.76041658930926;
points[1620].y = 78.06848030018762;
points[1620].color = giallo;

points[1621].x = 91.19643227681364;
points[1621].y = 81.2936210131332;
points[1621].color = blu;

points[1622].x = 106.76041658930926;
points[1622].y = 78.06848030018762;
points[1622].color = giallo;

points[1623].x = 90.20772118389166;
points[1623].y = 81.19911995855068;
points[1623].color = blu;

points[1624].x = 105.77512138895628;
points[1624].y = 78.18105065666042;
points[1624].color = giallo;

points[1625].x = 89.21113707646009;
points[1625].y = 81.18105065666042;
points[1625].color = blu;

points[1626].x = 104.77512138895628;
points[1626].y = 78.18105065666042;
points[1626].color = giallo;

points[1627].x = 88.21114346194584;
points[1627].y = 81.18039991508802;
points[1627].color = blu;

points[1628].x = 103.77512138895628;
points[1628].y = 78.18105065666042;
points[1628].color = giallo;

points[1629].x = 87.22584187610659;
points[1629].y = 81.06848030018762;
points[1629].color = blu;

points[1630].x = 102.77512138895628;
points[1630].y = 78.18105065666042;
points[1630].color = giallo;

points[1631].x = 86.24054667575327;
points[1631].y = 80.95590994371481;
points[1631].color = blu;

points[1632].x = 101.77512138895628;
points[1632].y = 78.18105065666042;
points[1632].color = giallo;

points[1633].x = 85.25525147539975;
points[1633].y = 80.84333958724203;
points[1633].color = blu;

points[1634].x = 100.78982618860276;
points[1634].y = 78.2936210131332;
points[1634].color = giallo;

points[1635].x = 84.26995627504625;
points[1635].y = 80.73076923076923;
points[1635].color = blu;

points[1636].x = 99.78982618860276;
points[1636].y = 78.2936210131332;
points[1636].color = giallo;

points[1637].x = 83.2846610746926;
points[1637].y = 80.61819887429644;
points[1637].color = blu;

points[1638].x = 98.78982618860276;
points[1638].y = 78.2936210131332;
points[1638].color = giallo;

points[1639].x = 82.30295930926304;
points[1639].y = 80.47028165144398;
points[1639].color = blu;

points[1640].x = 97.78982618860276;
points[1640].y = 78.2936210131332;
points[1640].color = giallo;

points[1641].x = 81.32566288129628;
points[1641].y = 80.30261000168099;
points[1641].color = blu;

points[1642].x = 96.78982618860276;
points[1642].y = 78.2936210131332;
points[1642].color = giallo;

points[1643].x = 80.34472481302336;
points[1643].y = 80.16107093171195;
points[1643].color = blu;

points[1644].x = 95.78982618860276;
points[1644].y = 78.2936210131332;
points[1644].color = giallo;

points[1645].x = 79.37899944611796;
points[1645].y = 79.92824541631249;
points[1645].color = blu;

points[1646].x = 94.78982618860276;
points[1646].y = 78.2936210131332;
points[1646].color = giallo;

points[1647].x = 78.40909886255359;
points[1647].y = 79.70853133306112;
points[1647].color = blu;

points[1648].x = 93.78982618860276;
points[1648].y = 78.2936210131332;
points[1648].color = giallo;

points[1649].x = 77.4465731099282;
points[1649].y = 79.45355525354778;
points[1649].color = blu;

points[1650].x = 92.78982618860276;
points[1650].y = 78.2936210131332;
points[1650].color = giallo;

points[1651].x = 76.48243980542959;
points[1651].y = 79.21216609619142;
points[1651].color = blu;

points[1652].x = 91.78982618860276;
points[1652].y = 78.2936210131332;
points[1652].color = giallo;

points[1653].x = 75.5365877633495;
points[1653].y = 78.91292866366477;
points[1653].color = blu;

points[1654].x = 90.8040591930533;
points[1654].y = 78.19249653153736;
points[1654].color = giallo;

points[1655].x = 74.59884228132093;
points[1655].y = 78.58007494788238;
points[1655].color = blu;

points[1656].x = 89.8045309882491;
points[1656].y = 78.18105065666042;
points[1656].color = giallo;

points[1657].x = 73.64178017096174;
points[1657].y = 78.30397603389554;
points[1657].color = blu;

points[1658].x = 88.8045309882491;
points[1658].y = 78.18105065666042;
points[1658].color = giallo;

points[1659].x = 72.72416516107447;
points[1659].y = 77.92096784978452;
points[1659].color = blu;

points[1660].x = 87.81923578789586;
points[1660].y = 78.06848030018762;
points[1660].color = giallo;

points[1661].x = 71.80735452135373;
points[1661].y = 77.53732888069192;
points[1661].color = blu;

points[1662].x = 86.83393489833516;
points[1662].y = 77.95651025483475;
points[1662].color = giallo;

points[1663].x = 70.90037663223241;
points[1663].y = 77.12365916250063;
points[1663].color = blu;

points[1664].x = 85.84864538718887;
points[1664].y = 77.84333958724203;
points[1664].color = giallo;

points[1665].x = 70.00072508549795;
points[1665].y = 76.70176127769838;
points[1665].color = blu;

points[1666].x = 84.86335018683565;
points[1666].y = 77.73076923076923;
points[1666].color = giallo;

points[1667].x = 69.11235748252852;
points[1667].y = 76.26082933978685;
points[1667].color = blu;

points[1668].x = 83.87790755268908;
points[1668].y = 77.62346978643214;
points[1668].color = giallo;

points[1669].x = 68.24061608902092;
points[1669].y = 75.79306279498932;
points[1669].color = blu;

points[1670].x = 82.903961116587;
points[1670].y = 77.4446192278195;
points[1670].color = giallo;

points[1671].x = 67.37758324372598;
points[1671].y = 75.29523239004133;
points[1671].color = blu;

points[1672].x = 81.92262818583457;
points[1672].y = 77.29342770246953;
points[1672].color = giallo;

points[1673].x = 66.53242883565102;
points[1673].y = 74.76884990782511;
points[1673].color = blu;

points[1674].x = 80.93845215145353;
points[1674].y = 77.16510637341949;
points[1674].color = giallo;

points[1675].x = 65.6925979653843;
points[1675].y = 74.24058142678932;
points[1675].color = blu;

points[1676].x = 79.9765514112158;
points[1676].y = 76.91666599251712;
points[1676].color = giallo;

points[1677].x = 64.86342281963063;
points[1677].y = 73.69297434658591;
points[1677].color = blu;

points[1678].x = 79.00426278831777;
points[1678].y = 76.7066982586805;
points[1678].color = giallo;

points[1679].x = 64.04716542839344;
points[1679].y = 73.1274644477545;
points[1679].color = blu;

points[1680].x = 78.04770958305505;
points[1680].y = 76.42734281069352;
points[1680].color = giallo;

points[1681].x = 63.23333942684589;
points[1681].y = 72.56140627769135;
points[1681].color = blu;

points[1682].x = 77.0991555133005;
points[1682].y = 76.14721033779563;
points[1682].color = giallo;

points[1683].x = 62.39521041685776;
points[1683].y = 72.03004428269944;
points[1683].color = blu;

points[1684].x = 76.13483287092878;
points[1684].y = 75.89852181759869;
points[1684].color = giallo;

points[1685].x = 61.56049725836311;
points[1685].y = 71.49179384096729;
points[1685].color = blu;

points[1686].x = 75.20165966148954;
points[1686].y = 75.55002227710159;
points[1686].color = giallo;

points[1687].x = 60.73760948516325;
points[1687].y = 70.93490023851416;
points[1687].color = blu;

points[1688].x = 74.27418059904147;
points[1688].y = 75.20402045298613;
points[1688].color = giallo;

points[1689].x = 59.92844091828771;
points[1689].y = 70.36120591475543;
points[1689].color = blu;

points[1690].x = 73.35349312415201;
points[1690].y = 74.83149596112654;
points[1690].color = giallo;

points[1691].x = 59.100118859022146;
points[1691].y = 69.81467838749637;
points[1691].color = blu;

points[1692].x = 72.42162424761403;
points[1692].y = 74.4796512630377;
points[1692].color = giallo;

points[1693].x = 58.26043973544831;
points[1693].y = 69.28609159536953;
points[1693].color = blu;

points[1694].x = 71.53001124516479;
points[1694].y = 74.04526508656657;
points[1694].color = giallo;

points[1695].x = 57.431449595549616;
points[1695].y = 68.73819348810318;
points[1695].color = blu;

points[1696].x = 70.64216777045145;
points[1696].y = 73.60360664189636;
points[1696].color = giallo;

points[1697].x = 56.61540930335788;
points[1697].y = 68.1724183001782;
points[1697].color = blu;

points[1698].x = 69.7568821509215;
points[1698].y = 73.14558719559895;
points[1698].color = giallo;

points[1699].x = 55.80116273225786;
points[1699].y = 67.60695030228194;
points[1699].color = blu;

points[1700].x = 68.90268263447298;
points[1700].y = 72.63510749791743;
points[1700].color = giallo;

points[1701].x = 54.96297142921018;
points[1701].y = 67.0757112783287;
points[1701].color = blu;

points[1702].x = 68.04782112976949;
points[1702].y = 72.1229910607511;
points[1702].color = giallo;

points[1703].x = 54.12842925090975;
points[1703].y = 66.53715242975028;
points[1703].color = blu;

points[1704].x = 67.21028094909893;
points[1704].y = 71.59043335639805;
points[1704].color = giallo;

points[1705].x = 53.317826279431884;
points[1705].y = 65.95961032416491;
points[1705].color = blu;

points[1706].x = 66.40064113400177;
points[1706].y = 71.01837108261427;
points[1706].color = giallo;

points[1707].x = 52.49417176044617;
points[1707].y = 65.40348459931539;
points[1707].color = blu;

points[1708].x = 65.58220606005591;
points[1708].y = 70.45557560403239;
points[1708].color = giallo;

points[1709].x = 51.6871127607374;
points[1709].y = 64.8215663125522;
points[1709].color = blu;

points[1710].x = 64.75120019692872;
points[1710].y = 69.9109228226703;
points[1710].color = giallo;

points[1711].x = 50.838755059170765;
points[1711].y = 64.29922427966174;
points[1711].color = blu;

points[1712].x = 63.91173573920332;
points[1712].y = 69.38207951710388;
points[1712].color = giallo;

points[1713].x = 50.01050809571725;
points[1713].y = 63.75174096063545;
points[1713].color = blu;

points[1714].x = 63.088649155371094;
points[1714].y = 68.82849203882833;
points[1714].color = giallo;

points[1715].x = 49.18398424846836;
points[1715].y = 63.20006721929053;
points[1715].color = blu;

points[1716].x = 62.2771787354875;
points[1716].y = 68.25736716901227;
points[1716].color = giallo;

points[1717].x = 48.38974752581459;
points[1717].y = 62.6035213667005;
points[1717].color = blu;

points[1718].x = 61.45208277475758;
points[1718].y = 67.70359736290263;
points[1718].color = giallo;

points[1719].x = 47.54780820383005;
points[1719].y = 62.07120756376149;
points[1719].color = blu;

points[1720].x = 60.615557854916304;
points[1720].y = 67.16871037146414;
points[1720].color = giallo;

points[1721].x = 46.72021302655429;
points[1721].y = 61.521149073650314;
points[1721].color = blu;

points[1722].x = 59.77807310849832;
points[1722].y = 66.63603702889725;
points[1722].color = giallo;

points[1723].x = 45.9107556171953;
points[1723].y = 60.9419786965159;
points[1723].color = blu;

points[1724].x = 58.968849465237376;
points[1724].y = 66.06336415149963;
points[1724].color = giallo;

points[1725].x = 45.09589319414812;
points[1725].y = 60.369166958455736;
points[1725].color = blu;

points[1726].x = 58.150196589363695;
points[1726].y = 65.50084600183786;
points[1726].color = giallo;

points[1727].x = 44.270607608335965;
points[1727].y = 59.812475582301424;
points[1727].color = blu;

points[1728].x = 57.31900851030063;
points[1728].y = 64.95649512503432;
points[1728].color = giallo;

points[1729].x = 43.45946543810571;
points[1729].y = 59.23579870046476;
points[1729].color = blu;

points[1730].x = 56.47961248071576;
points[1730].y = 64.42752839167838;
points[1730].color = giallo;

points[1731].x = 42.631821685770895;
points[1731].y = 58.67928215272208;
points[1731].color = blu;

points[1732].x = 55.65697152963533;
points[1732].y = 63.8733641611865;
points[1732].color = giallo;

points[1733].x = 41.82286611225003;
points[1733].y = 58.09947192383859;
points[1733].color = blu;

points[1734].x = 54.842382552935035;
points[1734].y = 63.30024125982045;
points[1734].color = giallo;

points[1735].x = 41.01421856604009;
points[1735].y = 57.519287826253844;
points[1735].color = blu;

points[1736].x = 54.01683830222577;
points[1736].y = 62.747121988403485;
points[1736].color = giallo;

points[1737].x = 40.20588571008803;
points[1737].y = 56.93873545029003;
points[1737].color = blu;

points[1738].x = 53.205592930085054;
points[1738].y = 62.17061923991282;
points[1738].color = giallo;

points[1739].x = 39.44676660515869;
points[1739].y = 56.296041920575746;
points[1739].color = blu;

points[1740].x = 52.361723335531074;
points[1740].y = 61.642064614960425;
points[1740].color = giallo;

points[1741].x = 38.7526636449181;
points[1741].y = 55.5771931701322;
points[1741].color = blu;

points[1742].x = 51.53097609700232;
points[1742].y = 61.098211100534606;
points[1742].color = giallo;

points[1743].x = 38.33800557683604;
points[1743].y = 54.67331259206364;
points[1743].color = blu;

points[1744].x = 50.723097083630186;
points[1744].y = 60.52310294173538;
points[1744].color = giallo;

points[1745].x = 38.12476547842402;
points[1745].y = 53.712053125436704;
points[1745].color = blu;

points[1746].x = 49.90537281888215;
points[1746].y = 59.95711688297893;
points[1746].color = giallo;

points[1747].x = 38.12476547842402;
points[1747].y = 52.712053125436704;
points[1747].color = blu;

points[1748].x = 49.06574464688505;
points[1748].y = 59.42097214572691;
points[1748].color = giallo;

points[1749].x = 38.31434819482727;
points[1749].y = 51.73945219662461;
points[1749].color = blu;

points[1750].x = 48.25872956513564;
points[1750].y = 58.84492828704473;
points[1750].color = giallo;

points[1751].x = 38.60530808691872;
points[1751].y = 50.78708301108234;
points[1751].color = blu;

points[1752].x = 47.43545692711356;
points[1752].y = 58.28240767062572;
points[1752].color = giallo;

points[1753].x = 39.00803210416834;
points[1753].y = 49.87611037252966;
points[1753].color = blu;

points[1754].x = 46.624887860813736;
points[1754].y = 57.70481344903822;
points[1754].color = giallo;

points[1755].x = 39.5056788417988;
points[1755].y = 49.01296159753451;
points[1755].color = blu;

points[1756].x = 45.79086171356668;
points[1756].y = 57.15922242066369;
points[1756].color = giallo;

points[1757].x = 40.191714527439004;
points[1757].y = 48.28734036352292;
points[1757].color = blu;

points[1758].x = 44.98025927482891;
points[1758].y = 56.58166430347281;
points[1758].color = giallo;

points[1759].x = 40.915747359501275;
points[1759].y = 47.59938273887588;
points[1759].color = blu;

points[1760].x = 44.17147353674761;
points[1760].y = 56.00164609403646;
points[1760].color = giallo;

points[1761].x = 41.80272842811965;
points[1761].y = 47.14071468433581;
points[1761].color = blu;

points[1762].x = 43.34488162427024;
points[1762].y = 55.44363731241694;
points[1762].color = giallo;

points[1763].x = 42.75503842133766;
points[1763].y = 46.886491557223266;
points[1763].color = blu;

points[1764].x = 42.53387824810461;
points[1764].y = 54.8667305693845;
points[1764].color = giallo;

points[1765].x = 43.75503842133766;
points[1765].y = 46.886491557223266;
points[1765].color = blu;

points[1766].x = 41.74330627043449;
points[1766].y = 54.26388042347162;
points[1766].color = giallo;

points[1767].x = 44.75503842133766;
points[1767].y = 46.886491557223266;
points[1767].color = blu;

points[1768].x = 41.15719665032213;
points[1768].y = 53.50464311458429;
points[1768].color = giallo;

points[1769].x = 45.75503842133766;
points[1769].y = 46.886491557223266;
points[1769].color = blu;

points[1770].x = 41.284225657413906;
points[1770].y = 52.540297680173346;
points[1770].color = giallo;

points[1771].x = 46.74033362169145;
points[1771].y = 46.773921200750465;
points[1771].color = blu;

points[1772].x = 41.60569228068423;
points[1772].y = 51.60172142275632;
points[1772].color = giallo;

points[1773].x = 47.74033362169145;
points[1773].y = 46.773921200750465;
points[1773].color = blu;

points[1774].x = 42.048333770515086;
points[1774].y = 50.71449157968721;
points[1774].color = giallo;

points[1775].x = 48.74033362169145;
points[1775].y = 46.773921200750465;
points[1775].color = blu;

points[1776].x = 42.782457572993465;
points[1776].y = 50.060626953146794;
points[1776].color = giallo;

points[1777].x = 49.74033362169145;
points[1777].y = 46.773921200750465;
points[1777].color = blu;

points[1778].x = 43.739904031201306;
points[1778].y = 49.886491557223266;
points[1778].color = giallo;

points[1779].x = 50.74033362169145;
points[1779].y = 46.773921200750465;
points[1779].color = blu;

points[1780].x = 44.739904031201306;
points[1780].y = 49.886491557223266;
points[1780].color = giallo;

points[1781].x = 51.74033362169145;
points[1781].y = 46.773921200750465;
points[1781].color = blu;

points[1782].x = 45.739904031201306;
points[1782].y = 49.886491557223266;
points[1782].color = giallo;

points[1783].x = 52.74033362169145;
points[1783].y = 46.773921200750465;
points[1783].color = blu;

points[1784].x = 46.73643103043351;
points[1784].y = 49.843275494205386;
points[1784].color = giallo;

points[1785].x = 53.74033362169145;
points[1785].y = 46.773921200750465;
points[1785].color = blu;

points[1786].x = 47.7251992315551;
points[1786].y = 49.773921200750465;
points[1786].color = giallo;

points[1787].x = 54.74033362169145;
points[1787].y = 46.773921200750465;
points[1787].color = blu;

points[1788].x = 48.7251992315551;
points[1788].y = 49.773921200750465;
points[1788].color = giallo;

points[1789].x = 55.74033362169145;
points[1789].y = 46.773921200750465;
points[1789].color = blu;

points[1790].x = 49.7251992315551;
points[1790].y = 49.773921200750465;
points[1790].color = giallo;

points[1791].x = 56.74033362169145;
points[1791].y = 46.773921200750465;
points[1791].color = blu;

points[1792].x = 50.7251992315551;
points[1792].y = 49.773921200750465;
points[1792].color = giallo;

points[1793].x = 57.72909474831238;
points[1793].y = 46.70450881540579;
points[1793].color = blu;

points[1794].x = 51.7251992315551;
points[1794].y = 49.773921200750465;
points[1794].color = giallo;

points[1795].x = 58.725628822045564;
points[1795].y = 46.66135084427767;
points[1795].color = blu;

points[1796].x = 52.7251992315551;
points[1796].y = 49.773921200750465;
points[1796].color = giallo;

points[1797].x = 59.725628822045564;
points[1797].y = 46.66135084427767;
points[1797].color = blu;

points[1798].x = 53.7251992315551;
points[1798].y = 49.773921200750465;
points[1798].color = giallo;

points[1799].x = 60.721318939996;
points[1799].y = 46.71123555054764;
points[1799].color = blu;

points[1800].x = 54.7251992315551;
points[1800].y = 49.773921200750465;
points[1800].color = giallo;

points[1801].x = 61.710924022399865;
points[1801].y = 46.773921200750465;
points[1801].color = blu;

points[1802].x = 55.7251992315551;
points[1802].y = 49.773921200750465;
points[1802].color = giallo;

points[1803].x = 62.710924022399865;
points[1803].y = 46.773921200750465;
points[1803].color = blu;

points[1804].x = 56.7251992315551;
points[1804].y = 49.773921200750465;
points[1804].color = giallo;

points[1805].x = 63.710924022399865;
points[1805].y = 46.773921200750465;
points[1805].color = blu;

points[1806].x = 57.7251992315551;
points[1806].y = 49.773921200750465;
points[1806].color = giallo;

points[1807].x = 64.71092402239987;
points[1807].y = 46.773921200750465;
points[1807].color = blu;

points[1808].x = 58.710494431909325;
points[1808].y = 49.66135084427767;
points[1808].color = giallo;

points[1809].x = 65.71092402239987;
points[1809].y = 46.773921200750465;
points[1809].color = blu;

points[1810].x = 59.710494431909325;
points[1810].y = 49.66135084427767;
points[1810].color = giallo;

points[1811].x = 66.71092402239987;
points[1811].y = 46.773921200750465;
points[1811].color = blu;

points[1812].x = 60.695789632263455;
points[1812].y = 49.773921200750465;
points[1812].color = giallo;

points[1813].x = 67.71092402239987;
points[1813].y = 46.773921200750465;
points[1813].color = blu;

points[1814].x = 61.695789632263455;
points[1814].y = 49.773921200750465;
points[1814].color = giallo;

points[1815].x = 68.71092402239987;
points[1815].y = 46.773921200750465;
points[1815].color = blu;

points[1816].x = 62.695789632263455;
points[1816].y = 49.773921200750465;
points[1816].color = giallo;

points[1817].x = 69.71092402239987;
points[1817].y = 46.773921200750465;
points[1817].color = blu;

points[1818].x = 63.695789632263455;
points[1818].y = 49.773921200750465;
points[1818].color = giallo;

points[1819].x = 70.71092402239987;
points[1819].y = 46.773921200750465;
points[1819].color = blu;

points[1820].x = 64.69578963226346;
points[1820].y = 49.773921200750465;
points[1820].color = giallo;

points[1821].x = 71.71092402239987;
points[1821].y = 46.773921200750465;
points[1821].color = blu;

points[1822].x = 65.69578963226346;
points[1822].y = 49.773921200750465;
points[1822].color = giallo;

points[1823].x = 72.71092402239987;
points[1823].y = 46.773921200750465;
points[1823].color = blu;

points[1824].x = 66.69578963226346;
points[1824].y = 49.773921200750465;
points[1824].color = giallo;

points[1825].x = 73.71092402239987;
points[1825].y = 46.773921200750465;
points[1825].color = blu;

points[1826].x = 67.69578963226346;
points[1826].y = 49.773921200750465;
points[1826].color = giallo;

points[1827].x = 74.71092402239987;
points[1827].y = 46.773921200750465;
points[1827].color = blu;

points[1828].x = 68.69578963226346;
points[1828].y = 49.773921200750465;
points[1828].color = giallo;

points[1829].x = 75.71092402239987;
points[1829].y = 46.773921200750465;
points[1829].color = blu;

points[1830].x = 69.69578963226346;
points[1830].y = 49.773921200750465;
points[1830].color = giallo;

points[1831].x = 76.71092402239987;
points[1831].y = 46.773921200750465;
points[1831].color = blu;

points[1832].x = 70.69578963226346;
points[1832].y = 49.773921200750465;
points[1832].color = giallo;

points[1833].x = 77.71092402239987;
points[1833].y = 46.773921200750465;
points[1833].color = blu;

points[1834].x = 71.69578963226346;
points[1834].y = 49.773921200750465;
points[1834].color = giallo;

points[1835].x = 78.71092402239987;
points[1835].y = 46.773921200750465;
points[1835].color = blu;

points[1836].x = 72.69578963226346;
points[1836].y = 49.773921200750465;
points[1836].color = giallo;

points[1837].x = 79.71092402239987;
points[1837].y = 46.773921200750465;
points[1837].color = blu;

points[1838].x = 73.69578963226346;
points[1838].y = 49.773921200750465;
points[1838].color = giallo;

points[1839].x = 80.71092402239987;
points[1839].y = 46.773921200750465;
points[1839].color = blu;

points[1840].x = 74.69578963226346;
points[1840].y = 49.773921200750465;
points[1840].color = giallo;

points[1841].x = 81.71092402239987;
points[1841].y = 46.773921200750465;
points[1841].color = blu;

points[1842].x = 75.69578963226346;
points[1842].y = 49.773921200750465;
points[1842].color = giallo;

points[1843].x = 82.71092402239987;
points[1843].y = 46.773921200750465;
points[1843].color = blu;

points[1844].x = 76.69578963226346;
points[1844].y = 49.773921200750465;
points[1844].color = giallo;

points[1845].x = 83.71092402239987;
points[1845].y = 46.773921200750465;
points[1845].color = blu;

points[1846].x = 77.69578963226346;
points[1846].y = 49.773921200750465;
points[1846].color = giallo;

points[1847].x = 84.69621922275286;
points[1847].y = 46.66135084427767;
points[1847].color = blu;

points[1848].x = 78.69578963226346;
points[1848].y = 49.773921200750465;
points[1848].color = giallo;

points[1849].x = 85.67986768133581;
points[1849].y = 46.54029983268446;
points[1849].color = blu;

points[1850].x = 79.69578963226346;
points[1850].y = 49.773921200750465;
points[1850].color = giallo;

points[1851].x = 86.65328794729234;
points[1851].y = 46.34480072439586;
points[1851].color = blu;

points[1852].x = 80.69578963226346;
points[1852].y = 49.773921200750465;
points[1852].color = giallo;

points[1853].x = 87.63739563559065;
points[1853].y = 46.211069418386494;
points[1853].color = blu;

points[1854].x = 81.69578963226346;
points[1854].y = 49.773921200750465;
points[1854].color = giallo;

points[1855].x = 88.59481197583045;
points[1855].y = 45.941935924772324;
points[1855].color = blu;

points[1856].x = 82.69578963226346;
points[1856].y = 49.773921200750465;
points[1856].color = giallo;

points[1857].x = 89.54870311040399;
points[1857].y = 45.6698545050259;
points[1857].color = blu;

points[1858].x = 83.69578963226346;
points[1858].y = 49.773921200750465;
points[1858].color = giallo;

points[1859].x = 90.5117729373396;
points[1859].y = 45.41801763369577;
points[1859].color = blu;

points[1860].x = 84.69436558914737;
points[1860].y = 49.75003650683736;
points[1860].color = giallo;

points[1861].x = 91.44436818774291;
points[1861].y = 45.067906981371394;
points[1861].color = blu;

points[1862].x = 85.68093086979637;
points[1862].y = 49.65592238288201;
points[1862].color = giallo;

points[1863].x = 92.37244638097165;
points[1863].y = 44.708388584074946;
points[1863].color = blu;

points[1864].x = 86.65412961229681;
points[1864].y = 49.4490477545718;
points[1864].color = giallo;

points[1865].x = 93.28528894612364;
points[1865].y = 44.309331382563386;
points[1865].color = blu;

points[1866].x = 87.63660311457991;
points[1866].y = 49.31403248873182;
points[1866].color = giallo;

points[1867].x = 94.19672727998788;
points[1867].y = 43.907919365288556;
points[1867].color = blu;

points[1868].x = 88.61412751680852;
points[1868].y = 49.13502265822088;
points[1868].color = giallo;

points[1869].x = 95.09018259795812;
points[1869].y = 43.46140976024638;
points[1869].color = blu;

points[1870].x = 89.57117598932524;
points[1870].y = 48.86123079599487;
points[1870].color = giallo;

points[1871].x = 95.95772769718515;
points[1871].y = 42.971589145819095;
points[1871].color = blu;

points[1872].x = 90.52807927937292;
points[1872].y = 48.58904475594634;
points[1872].color = giallo;

points[1873].x = 96.82666883404545;
points[1873].y = 42.48492209720261;
points[1873].color = blu;

points[1874].x = 91.48598704857875;
points[1874].y = 48.326642473551196;
points[1874].color = giallo;

points[1875].x = 97.65190052896743;
points[1875].y = 41.934096200643644;
points[1875].color = blu;

points[1876].x = 92.41854876983146;
points[1876].y = 47.97644568809216;
points[1876].color = giallo;

points[1877].x = 98.45898786072553;
points[1877].y = 41.35220638482248;
points[1877].color = blu;

points[1878].x = 93.35081777261763;
points[1878].y = 47.62552321908155;
points[1878].color = giallo;

points[1879].x = 99.25785956442833;
points[1879].y = 40.75835487345373;
points[1879].color = blu;

points[1880].x = 94.26390409336358;
points[1880].y = 47.22696544487962;
points[1880].color = giallo;

points[1881].x = 100.02354343360255;
points[1881].y = 40.125305078372506;
points[1881].color = blu;

points[1882].x = 95.17601368974258;
points[1882].y = 46.82692625318689;
points[1882].color = giallo;

points[1883].x = 100.73101271619424;
points[1883].y = 39.41856722147165;
points[1883].color = blu;

points[1884].x = 96.06923232921226;
points[1884].y = 46.37993208305533;
points[1884].color = giallo;

points[1885].x = 101.43690615994447;
points[1885].y = 38.711942352030576;
points[1885].color = blu;

points[1886].x = 96.96243381492356;
points[1886].y = 45.932899280942685;
points[1886].color = giallo;

points[1887].x = 102.10101676034793;
points[1887].y = 37.97188799899267;
points[1887].color = blu;

points[1888].x = 97.83069988078066;
points[1888].y = 45.44456369594922;
points[1888].color = giallo;

points[1889].x = 102.73274645431539;
points[1889].y = 37.20354511995205;
points[1889].color = blu;

points[1890].x = 98.67693706219406;
points[1890].y = 44.91875595977038;
points[1890].color = giallo;

points[1891].x = 103.31266577305561;
points[1891].y = 36.39467822437455;
points[1891].color = blu;

points[1892].x = 99.48489924980701;
points[1892].y = 44.34373816692335;
points[1892].color = giallo;

points[1893].x = 103.85945919292011;
points[1893].y = 35.56147637257326;
points[1893].color = blu;

points[1894].x = 100.29295568465517;
points[1894].y = 43.76320951082025;
points[1894].color = giallo;

points[1895].x = 104.34521108485404;
points[1895].y = 34.692047913314006;
points[1895].color = blu;

points[1896].x = 101.0986899886566;
points[1896].y = 43.17897048333915;
points[1896].color = giallo;

points[1897].x = 104.79134468680193;
points[1897].y = 33.800414825652275;
points[1897].color = blu;

points[1898].x = 101.85654215444251;
points[1898].y = 42.53494704465184;
points[1898].color = giallo;

points[1899].x = 105.16523071758988;
points[1899].y = 32.87832184029033;
points[1899].color = blu;

points[1900].x = 102.5699203810007;
points[1900].y = 41.836489346331895;
points[1900].color = giallo;

points[1901].x = 105.513022311987;
points[1901].y = 31.94492298539243;
points[1901].color = blu;

points[1902].x = 103.3173192225242;
points[1902].y = 41.18241388499965;
points[1902].color = giallo;

points[1903].x = 105.80191950804218;
points[1903].y = 30.991742819125086;
points[1903].color = blu;

points[1904].x = 103.96409620315812;
points[1904].y = 40.42739299593623;
points[1904].color = giallo;

points[1905].x = 106.00478675543117;
points[1905].y = 30.020922214763402;
points[1905].color = blu;

points[1906].x = 104.63383890076369;
points[1906].y = 39.688798584485724;
points[1906].color = giallo;

points[1907].x = 106.18106147239875;
points[1907].y = 29.0418740314792;
points[1907].color = blu;

points[1908].x = 105.25341594762261;
points[1908].y = 38.912932538526135;
points[1908].color = giallo;

points[1909].x = 106.42650168956243;
points[1909].y = 28.078916083878227;
points[1909].color = blu;

points[1910].x = 105.83162974375092;
points[1910].y = 38.10293365389575;
points[1910].color = giallo;

points[1911].x = 106.56754221388367;
points[1911].y = 27.098858290135375;
points[1911].color = blu;

points[1912].x = 106.39341181760926;
points[1912].y = 37.27910303196783;
points[1912].color = giallo;

points[1913].x = 106.68011257035647;
points[1913].y = 26.113563089780946;
points[1913].color = blu;

points[1914].x = 106.8903363015181;
points[1914].y = 36.415503749545564;
points[1914].color = giallo;

points[1915].x = 106.59159947007235;
points[1915].y = 25.126828185803028;
points[1915].color = blu;

points[1916].x = 107.38270507110127;
points[1916].y = 35.549277059276505;
points[1916].color = giallo;

points[1917].x = 106.48424857140543;
points[1917].y = 24.14103884097928;
points[1917].color = blu;

points[1918].x = 107.77874168438343;
points[1918].y = 34.635075851547214;
points[1918].color = giallo;

points[1919].x = 106.34570574852468;
points[1919].y = 23.157604376743205;
points[1919].color = blu;

points[1920].x = 108.13374980887552;
points[1920].y = 33.70483745078919;
points[1920].color = giallo;

points[1921].x = 106.08833834717773;
points[1921].y = 22.201208142408262;
points[1921].color = blu;

points[1922].x = 108.47570079784103;
points[1922].y = 32.76843787195821;
points[1922].color = giallo;

points[1923].x = 105.82171839525859;
points[1923].y = 21.24513020844087;
points[1923].color = blu;

points[1924].x = 108.74743927675776;
points[1924].y = 31.809621910276306;
points[1924].color = giallo;

points[1925].x = 105.46301386491648;
points[1925].y = 20.316915910760724;
points[1925].color = blu;

points[1926].x = 108.98467357647468;
points[1926].y = 30.84456906590656;
points[1926].color = giallo;

points[1927].x = 105.02315506296328;
points[1927].y = 19.4284373251646;
points[1927].color = blu;

points[1928].x = 109.14296307979893;
points[1928].y = 29.86513373586629;
points[1928].color = giallo;

points[1929].x = 104.51650748969018;
points[1929].y = 18.57178030887315;
points[1929].color = blu;

points[1930].x = 109.38874947374065;
points[1930].y = 28.902289529589005;
points[1930].color = giallo;

points[1931].x = 103.96392500793664;
points[1931].y = 17.745870196032087;
points[1931].color = blu;

points[1932].x = 109.54674449209132;
points[1932].y = 27.922441069686798;
points[1932].color = giallo;

points[1933].x = 103.34789787655505;
points[1933].y = 16.96694386693993;
points[1933].color = blu;

points[1934].x = 109.65444077749186;
points[1934].y = 26.93671541486471;
points[1934].color = giallo;

points[1935].x = 102.70160238843084;
points[1935].y = 16.211290461821633;
points[1935].color = blu;

points[1936].x = 109.68011257035647;
points[1936].y = 25.938302663325533;
points[1936].color = giallo;

points[1937].x = 101.96708963750343;
points[1937].y = 15.540395801618349;
points[1937].color = blu;

points[1938].x = 109.67973081965584;
points[1938].y = 24.93830554978883;
points[1938].color = giallo;

points[1939].x = 101.18768171073992;
points[1939].y = 14.921419261648332;
points[1939].color = blu;

points[1940].x = 109.56752467517359;
points[1940].y = 23.95300748987481;
points[1940].color = giallo;

points[1941].x = 100.38709810897909;
points[1941].y = 14.331838786542628;
points[1941].color = blu;

points[1942].x = 109.44125228034908;
points[1942].y = 22.96833143001311;
points[1942].color = giallo;

points[1943].x = 99.53064139112134;
points[1943].y = 13.825018872022255;
points[1943].color = blu;

points[1944].x = 109.255560690638;
points[1944].y = 21.992354059892897;
points[1944].color = giallo;

points[1945].x = 98.66397546711426;
points[1945].y = 13.333476828417501;
points[1945].color = blu;

points[1946].x = 108.96782127980778;
points[1946].y = 21.039099975877882;
points[1946].color = giallo;

points[1947].x = 97.75117970806909;
points[1947].y = 12.934793981484003;
points[1947].color = blu;

points[1948].x = 108.65514065456573;
points[1948].y = 20.096071516043004;
points[1948].color = giallo;

points[1949].x = 96.81740264346477;
points[1949].y = 12.588764036296496;
points[1949].color = blu;

points[1950].x = 108.30337070991976;
points[1950].y = 19.164169068522583;
points[1950].color = giallo;

points[1951].x = 95.87558406404303;
points[1951].y = 12.28077871489109;
points[1951].color = blu;

points[1952].x = 107.85719155509817;
points[1952].y = 18.272556995733908;
points[1952].color = giallo;

points[1953].x = 94.91270970577065;
points[1953].y = 12.027918920642286;
points[1953].color = blu;

points[1954].x = 107.36778518576945;
points[1954].y = 17.404807492219774;
points[1954].color = giallo;

points[1955].x = 93.93959737751808;
points[1955].y = 11.837711069418386;
points[1955].color = blu;

points[1956].x = 106.87372256983687;
points[1956].y = 16.539515796818808;
points[1956].color = giallo;

points[1957].x = 92.95430217716456;
points[1957].y = 11.72514071294559;
points[1957].color = blu;

points[1958].x = 106.26203087437231;
points[1958].y = 15.754218973589403;
points[1958].color = giallo;

points[1959].x = 91.95430217716456;
points[1959].y = 11.72514071294559;
points[1959].color = blu;

points[1960].x = 105.66813284880325;
points[1960].y = 14.95747130195188;
points[1960].color = giallo;

points[1961].x = 90.95430217716456;
points[1961].y = 11.72514071294559;
points[1961].color = blu;

points[1962].x = 104.97271112990813;
points[1962].y = 14.239758516179638;
points[1962].color = giallo;

points[1963].x = 89.96900573369362;
points[1963].y = 11.83749252072339;
points[1963].color = blu;

points[1964].x = 104.26560434872164;
points[1964].y = 13.532651734993136;
points[1964].color = giallo;

points[1965].x = 88.98337522362597;
points[1965].y = 11.941138386007488;
points[1965].color = blu;

points[1966].x = 103.508374120017;
points[1966].y = 12.887991862761298;
points[1966].color = giallo;

points[1967].x = 87.98371177645866;
points[1967].y = 11.950281425891182;
points[1967].color = blu;

points[1968].x = 102.73587388066191;
points[1968].y = 12.260497325627075;
points[1968].color = giallo;

points[1969].x = 86.98371177645866;
points[1969].y = 11.950281425891182;
points[1969].color = blu;

points[1970].x = 101.92564775533411;
points[1970].y = 11.682390882965532;
points[1970].color = giallo;

points[1971].x = 85.99841657610565;
points[1971].y = 12.062851782363978;
points[1971].color = blu;

points[1972].x = 101.06251956963868;
points[1972].y = 11.184711409512193;
points[1972].color = giallo;

points[1973].x = 84.99841657610565;
points[1973].y = 12.062851782363978;
points[1973].color = blu;

points[1974].x = 100.19671415788223;
points[1974].y = 10.691568464836193;
points[1974].color = giallo;

points[1975].x = 84.00927735129099;
points[1975].y = 12.129192092586052;
points[1975].color = blu;

points[1976].x = 99.29613352287267;
points[1976].y = 10.260629932216306;
points[1976].color = giallo;

points[1977].x = 83.01312137575269;
points[1977].y = 12.175422138836772;
points[1977].color = blu;

points[1978].x = 98.37388433903821;
points[1978].y = 9.886505776766125;
points[1978].color = giallo;

points[1979].x = 82.01312137575269;
points[1979].y = 12.175422138836772;
points[1979].color = blu;

points[1980].x = 97.44051169786661;
points[1980].y = 9.539286225243675;
points[1980].color = giallo;

points[1981].x = 81.01312137575269;
points[1981].y = 12.175422138836772;
points[1981].color = blu;

points[1982].x = 96.47981392476396;
points[1982].y = 9.2878967235055;
points[1982].color = giallo;

points[1983].x = 80.01312137575269;
points[1983].y = 12.175422138836772;
points[1983].color = blu;

points[1984].x = 95.52434859232801;
points[1984].y = 9.005269333570492;
points[1984].color = giallo;

points[1985].x = 79.01312137575269;
points[1985].y = 12.175422138836772;
points[1985].color = blu;

points[1986].x = 94.54397516011281;
points[1986].y = 8.840886481040211;
points[1986].color = giallo;

points[1987].x = 78.02782617539962;
points[1987].y = 12.062851782363978;
points[1987].color = blu;

points[1988].x = 93.55874872512865;
points[1988].y = 8.72520188861115;
points[1988].color = giallo;

points[1989].x = 77.02782617539962;
points[1989].y = 12.062851782363978;
points[1989].color = blu;

points[1990].x = 92.55874892002628;
points[1990].y = 8.72514071294559;
points[1990].color = giallo;

points[1991].x = 76.02782617539962;
points[1991].y = 12.062851782363978;
points[1991].color = blu;

points[1992].x = 91.55874892002628;
points[1992].y = 8.72514071294559;
points[1992].color = giallo;

points[1993].x = 75.02782617539962;
points[1993].y = 12.062851782363978;
points[1993].color = blu;

points[1994].x = 90.55874892002628;
points[1994].y = 8.72514071294559;
points[1994].color = giallo;

points[1995].x = 74.0425309750461;
points[1995].y = 11.950281425891182;
points[1995].color = blu;

points[1996].x = 89.56782168518092;
points[1996].y = 8.806902777653619;
points[1996].color = giallo;

points[1997].x = 73.0425309750461;
points[1997].y = 11.950281425891182;
points[1997].color = blu;

points[1998].x = 88.57681861953489;
points[1998].y = 8.88002856334763;
points[1998].color = giallo;

points[1999].x = 72.0425309750461;
points[1999].y = 11.950281425891182;
points[1999].color = blu;

points[2000].x = 87.58815851932019;
points[2000].y = 8.950281425891182;
points[2000].color = giallo;

points[2001].x = 71.0425309750461;
points[2001].y = 11.950281425891182;
points[2001].color = blu;

points[2002].x = 86.58815851932019;
points[2002].y = 8.950281425891182;
points[2002].color = giallo;

points[2003].x = 70.04258307457127;
points[2003].y = 11.94764721192959;
points[2003].color = blu;

points[2004].x = 85.60286331896718;
points[2004].y = 9.062851782363978;
points[2004].color = giallo;

points[2005].x = 69.05723577469269;
points[2005].y = 11.837711069418386;
points[2005].color = blu;

points[2006].x = 84.60286331896718;
points[2006].y = 9.062851782363978;
points[2006].color = giallo;

points[2007].x = 68.05723577469269;
points[2007].y = 11.837711069418386;
points[2007].color = blu;

points[2008].x = 83.60311090083817;
points[2008].y = 9.07029906463827;
points[2008].color = giallo;

points[2009].x = 67.05723577469269;
points[2009].y = 11.837711069418386;
points[2009].color = blu;

points[2010].x = 82.6175681186141;
points[2010].y = 9.175422138836772;
points[2010].color = giallo;

points[2011].x = 66.05723577469269;
points[2011].y = 11.837711069418386;
points[2011].color = blu;

points[2012].x = 81.6175681186141;
points[2012].y = 9.175422138836772;
points[2012].color = giallo;

points[2013].x = 65.06435253934366;
points[2013].y = 11.768113313007023;
points[2013].color = blu;

points[2014].x = 80.6175681186141;
points[2014].y = 9.175422138836772;
points[2014].color = giallo;

points[2015].x = 64.07194057433927;
points[2015].y = 11.72514071294559;
points[2015].color = blu;

points[2016].x = 79.6175681186141;
points[2016].y = 9.175422138836772;
points[2016].color = giallo;

points[2017].x = 63.071940574339266;
points[2017].y = 11.72514071294559;
points[2017].color = blu;

points[2018].x = 78.63227291826112;
points[2018].y = 9.062851782363978;
points[2018].color = giallo;

points[2019].x = 62.071940574339266;
points[2019].y = 11.72514071294559;
points[2019].color = blu;

points[2020].x = 77.63227291826112;
points[2020].y = 9.062851782363978;
points[2020].color = giallo;

points[2021].x = 61.081373859968444;
points[2021].y = 11.6412426066952;
points[2021].color = blu;

points[2022].x = 76.63227291826112;
points[2022].y = 9.062851782363978;
points[2022].color = giallo;

points[2023].x = 60.08664537398527;
points[2023].y = 11.612570356472796;
points[2023].color = blu;

points[2024].x = 75.63227291826112;
points[2024].y = 9.062851782363978;
points[2024].color = giallo;

points[2025].x = 59.08664537398527;
points[2025].y = 11.612570356472796;
points[2025].color = blu;

points[2026].x = 74.646977520948;
points[2026].y = 8.950343049644406;
points[2026].color = giallo;

points[2027].x = 58.08948344893576;
points[2027].y = 11.574784259258406;
points[2027].color = blu;

points[2028].x = 73.6469777179077;
points[2028].y = 8.950281425891182;
points[2028].color = giallo;

points[2029].x = 57.10135017363136;
points[2029].y = 11.5;
points[2029].color = blu;

points[2030].x = 72.6469777179077;
points[2030].y = 8.950281425891182;
points[2030].color = giallo;

points[2031].x = 56.10135017363136;
points[2031].y = 11.5;
points[2031].color = blu;

points[2032].x = 71.6469777179077;
points[2032].y = 8.950281425891182;
points[2032].color = giallo;

points[2033].x = 55.10135017363136;
points[2033].y = 11.5;
points[2033].color = blu;

points[2034].x = 70.6469777179077;
points[2034].y = 8.950281425891182;
points[2034].color = giallo;

points[2035].x = 54.10135017363136;
points[2035].y = 11.5;
points[2035].color = blu;

points[2036].x = 69.66168251755428;
points[2036].y = 8.837711069418386;
points[2036].color = giallo;

points[2037].x = 53.10135017363136;
points[2037].y = 11.5;
points[2037].color = blu;

scaleFactor = 15;
*/

	for (int i = 0; i < numPoints; i++){
		circlefill(
			screen,
			(int)(points[i].x * scaleFactor),
			(int)(points[i].y * scaleFactor),
			3,
			points[i].color
		);
	}

	// Test delaunay code
	// Delaunay triangulation
	trajectory_planning(0.0, 0.0, 0.0, points, trajectory);

	readkey(); // wait for a key press to close the window
	allegro_exit(); // deallocate graphics data structures
	return 0;

}

void trajectory_planning(float car_x, float car_y, float car_angle, cone *detected_cones, waypoint *trajectory)
{
	int N_detected_cones = numPoints;

	if (N_detected_cones < 3){
		return; // not enough cones to plan the trajectory
	}
	else {
		int connected_indices[N_detected_cones][2];
		const int B_idx = 0;
		const int Y_idx = 1;

		for (int i = 0; i < N_detected_cones; i++){
			for (int j = 0; j < 2; j++){
				connected_indices[i][j] = -1;
			}
		}

		for (int focus_idx = 0; focus_idx < N_detected_cones; focus_idx++)
		{
			float mDist_Y = 1000;
			float mDist_B = 1000;

			int mDist_Y_idx = -1;
			int mDist_B_idx = -1;

			// [TODO] Add a condition to check if the candidate index is free to be connected
			int focusColor = -1;

			if (detected_cones[focus_idx].color == giallo) 
				focusColor = Y_idx;
			else 
				focusColor = B_idx;

			if (connected_indices[focus_idx][B_idx] != -1 && connected_indices[focus_idx][Y_idx] != -1)
			{ 	// full connected
				continue;
			}
			
			if (connected_indices[focus_idx][Y_idx] == -1)
			{	// so search for the nearest yellow cone
				for (int candidate_idx = 0; candidate_idx < N_detected_cones; candidate_idx++){
					if (candidate_idx == focus_idx){
						continue;
					}
					
					if ((detected_cones[candidate_idx].color == giallo))// && (connected_indices[candidate_idx][focusColor] == -1))
					{
						float distance = sqrt(pow(detected_cones[candidate_idx].x - detected_cones[focus_idx].x, 2) + pow(detected_cones[candidate_idx].y - detected_cones[focus_idx].y, 2));
						
						if (distance < mDist_Y){
							mDist_Y = distance;
							mDist_Y_idx = candidate_idx;
						}
					}
				}
				// found the yellow one
				connected_indices[focus_idx][Y_idx] = mDist_Y_idx; 
				connected_indices[mDist_Y_idx][focusColor] = focus_idx;
			}

			if (connected_indices[focus_idx][B_idx] == -1)
			{	// so search for the nearest blue cone
				for (int candidate_idx = 0; candidate_idx < N_detected_cones; candidate_idx++){
					if (candidate_idx == focus_idx){
						continue;
					}

					if ((detected_cones[candidate_idx].color == blu))// && (connected_indices[candidate_idx][focusColor] == -1))
					{
						float distance = sqrt(pow(detected_cones[candidate_idx].x - detected_cones[focus_idx].x, 2) + pow(detected_cones[candidate_idx].y - detected_cones[focus_idx].y, 2));
						
						if (distance < mDist_B){
							mDist_B = distance;
							mDist_B_idx = candidate_idx;
						}
					}
				}
				// found the blue one
				connected_indices[focus_idx][B_idx] = mDist_B_idx; 
				connected_indices[mDist_B_idx][focusColor] = focus_idx;
			}

		}

		for (int i = 0; i < N_detected_cones; i++){
			if (connected_indices[i][B_idx] != -1 && connected_indices[i][Y_idx] != -1){
				line(
					screen,
					detected_cones[i].x * scaleFactor,
					detected_cones[i].y * scaleFactor,
					detected_cones[connected_indices[i][B_idx]].x * scaleFactor,
					detected_cones[connected_indices[i][B_idx]].y * scaleFactor,
					blu
				);
				line(
					screen,
					detected_cones[i].x * scaleFactor,
					detected_cones[i].y * scaleFactor,
					detected_cones[connected_indices[i][Y_idx]].x * scaleFactor,
					detected_cones[connected_indices[i][Y_idx]].y * scaleFactor,
					giallo
				);
				line(
					screen,
					detected_cones[connected_indices[i][B_idx]].x * scaleFactor,
					detected_cones[connected_indices[i][B_idx]].y * scaleFactor,
					detected_cones[connected_indices[i][Y_idx]].x * scaleFactor,
					detected_cones[connected_indices[i][Y_idx]].y * scaleFactor,
					giallo
				);
			}
		}

		for (int i = 0; i < N_detected_cones; i++){
			char c = detected_cones[i].color == giallo ? 'Y' : 'B';
			printf("\nCone %d (%c): ", i, c);
			for (int j = 0; j < 2; j++){
				char c = detected_cones[j].color == giallo ? 'Y' : 'B';
				printf("%d (%c)\t", connected_indices[i][j], c);
			}
		}



 	}
}

