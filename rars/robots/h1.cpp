/*
File name:	h1.cpp
Author:		Henning Klaskala
E-mail:		bm321465@muenchen.org
Robot name:	H1
Races:		all
Source code is:	public
Data file:	none
Date:		3 June 1998
Very fast on:	brands, cstlcomb, brazil, oval2, buenos, monza-76, mosport,
		nurburg, ra, silverst, spa, suzuka, tremblnt, watglen, zandvort,
		mlwaukee, indy500, nazareth, phoenix, loudon, michigan, pocono.

Sponsors please send money, diamonds, gold bars etc. to this address:	;-)
		Henning Klaskala
		Hildegardstrasse 18
		85716 Unterschleissheim
		Germany
*/
#include <string.h>
#include "car.h"
#include "track.h"
#define A(x)(x[i]*(f-j)+x[i+1]*(1-f+j))
#define N s.nearby[k]
#define E(x,y) else if(!strcmp(trackfile[current_track],x)||!strcmp(trackfile[current_track],y))
#ifndef DRIVERNAME
#define DRIVERNAME "H1"
#define FUNCTIONNAME H1
#endif
con_vec FUNCTIONNAME(situation&s)
{con_vec r;static int S=0,c=0,G=0,p=4;static double li=0;static track_desc T;
if(!c){c=1;my_name_is(DRIVERNAME);r.alpha=r.vc=0;return r;}
static unsigned char*v,*l="";
if(s.starting){T=get_track_description();S=0;G=0;li=0;p=4;if(0);
E("mlwaukee.trk","MLWAUKEE.TRK")p=8,
v="4EXMIZRNHKIRTPSXUQ34DEOLIKILSMQ[\\",
l="8/+-.1/1GTSZW[YT91..-,01JOZW[[[SH";
E("indy500.trk","INDY500.TRK")p=8,
v="),K5LBCZlfhkprptwfW^Whmjwqonnruv~_iSI9Damnkklw\200z\204_K=IRXdspmopnqqy",
l="A0,1872B]hjnonfYC:751/16CYfeje_R>310063?^hnie_UF2..0,52<J[kmojcWC";
E("nazareth.trk","NAZARETH.TRK")p=8,
v="NNGK<L]ajmltjui~ujYWGQQPUQIJRNW_[GEGZ_PSPJEKRRR_o",
l="5/,,./17ALUXZVRE90-/.-1<KSZWYZZO6.,,,,,/LQISSXYN8";
E("phoenix.trk","PHOENIX.TRK")p=8,
v=".7751WI@>@;>MIW`uScF?SZTeedv\213\220}u\201vp]VYHHQQGJJRQU]",
l="330-,,,-XXWZY[XN7/-,,/39DPWY[WRJ;51,.,5HUZSTXYYN6";
E("loudon.trk","LOUDON.TRK")p=8,
v="geegheJHAFEDBECE?R<@GSK`igjghkjkdeehgfIJFNF?@ABM@?GTIEh@ikkikhjgf",
l="9+0/-/--VW[WOJGE@B?===>FHIRX[ZTL;+-+..,/[YZ[RJGC?=:?8;>BFJS[[[UJ=";
E("michigan.trk","MICHIGAN.TRK")p=8,
v="\234\202t\217\231\200z\202\205\213\221\215\224\221\215\201qomszxtrutnv\207\216jsurjp\204\200\202st\200\226\177\244\212\230\240\241",
l="KNKKKLMOOOOKC;7ANV^^^_]TC6410/.2FV^UZ]]YKGMV_b`XL";
E("pocono.trk","POCONO.TRK")p=8,
v="!\"%\024\017&,3=6?6BBCTW@F*\03473/A@KAWKXNaII2=:G@DA;?KHPa[",
l="c6/5-/+4p\177\203{|\203\202{f;2</4+8w\212\203\210\200{shX80-,4,/q\215\214\213\214\215\211~h";
E("cstlcomb.trk","CSTLCOMB.TRK")
v="; \032Blokfu[(St}ga\\Q[;ZP^eH60\";008>0\";QORXRG6Mbccd_H+>9@:?<<8KNGSFF4\037?kkhmoW?:CFPOM",
l="ZmdT.)/7E_cQ50+2AEN_nooob\\J=3./6Vdjm,)+;OhkkmmieikeS5DQLEC78*06@Pmkd?7>BHflk++.>Q";
E("brands.trk","BRANDS.TRK")
v="\351\211*).!\036\'1:43BHEHHHLU_I/GWC959<=BOF,@[iZhlRBLCQ]YZVHPQSKVPJJTYcidYBN]X[VX[SGTZ`^XMHFLNONRP?)#03:@[CTBCFBHLSYcpE+Lk\\MPJLYS]XRZ_\\^JC",
l="_heY*+/)?47?Vdijhc`YI:-1Uee^ijd^D842,,16?EbhC3+/N\\fZ;3/>OQUKD>:EABGD75:<FGJ?.+20?GEK</)*2.,PigjdICVT2)+/:@71DZdfQID96-.8GRXZWXXR[";
E("brazil.trk","BRAZIL.TRK")
v="0\034K0\032\037\'@,,27C:5:>15>BAG0!\033!4A-#AKDPXI/-@>89UY_N?5078N9.0(*,9D8HLOVMZC9&(,\"\0350/<L7658G]CG$%+..K15Njhmaa3D]wcSX\033",
l="TA=OpnmqC+)3HSGGfokeH+-@qolT;*.Ckqnl^\\lZ+)3Dboi`B,)*>HJM]mqk=73.*).5AVeF/)+)D<9?jpq`B..6qqpb>0?TeoqfG?CJTm]Rc";
E("oval2.trk","OVAL2.TRK")
v="<68UGC:EMbJCL<IK`",
l="Q::<n\203\177\203fMHD^t\202\203^";
E("buenos.trk","BUENOS.TRK")
v=",=M2!$$&JN`OOD1APZ\\ULY:-+\037!3>84C;:FFWQHENE<9<:6DCDCAWS_]_S))%\"#2D&5R^?QW[GRA;958?=7B?DIC,//3@AA\' \030\025\03793.6-40;8C8>KPLHX4@<0)5:=4\':]VY^g",
l="ZU`\\/-*5337?W_XJ*++05?3=jjjgE,,?bjfS0-.1@AH<-++5I`ea8)*3PT_C*).)GVX[Y\\LE1-,).9HTghgWB9,Rbfj`=+47jeif^\\N:-+*2NW[SE>>>EZ^L)+),BSO[eggca";
E("monza-76.trk","MONZA-76.TRK")
v="\030*3I\327\337\326\341\334USVR*$06(\02743HMSoxpeuqrtcjhb~\201\201}~G007;\364\372\372\371\365\372\372CE:HIDTMF,JLXU^YO2LeoopoN$.MEGL+34DGVTPilmnng\0379A=9GTLV`d",
l="WnsP8<45CJNlvjhN10/57P`kyrduKT<B>EIGA6/9H<3.yqr]IB8-=_ss;)+:_omg92)1GWafmmpqnC85yxqjSK65*)0<nsrqaUhn1)3/;60BV";
E("mosport.trk","MOSPORT.TRK")
v="\037-7Xwqrps_D\\y|~z}[HQKOTSVE88<LTJM<97>;CDM<$;SILPVE\034DFIDQZE8GF7=G[TR[dX3KQDMRMF>)63&/6!&.:;9GD84HSRWj_",
l="v]JFEDBDFJOE<=AHLJSN0+)8GWWry\203\203\201pc]E,)*4V{~\200Q1*)=H;1s}\203\203ku}u0..)--+,5714\203z\200r_X_L;AGOke_Q87<HYpuw|}|zq";
E("nurburg.trk","NURBURG.TRK")
v="619-;5.HJ.\021*7-;2:A@EK9\0319?7>E>/>/6,&4:_|_2),8K,\032<KHKNXTTWXB1.,523B?)3CDFKOXKUhjkimP.PNAHNRRYOR",
l="Rfmg-+*6_v\200\201+)*Hb~\203}Z71/v\200\200\200fe]O)),Bb~\200\177-*))/9-/y~~f?++/?A2,\201}xkSDIE;)+8_x{d6.,.8E]T\202\201\177gK67;/";
E("ra.trk","RA.TRK")
v="fdh379@sgH!7+5:nbbgk\364\367\366\371\365\367\363\365\360\372\365y\031\'.(-4.5=4=2848lelpjxnvu\372\372\0364125ngj);DELP\370\370\366\365\362\370\367\367\365\364\364\361\365\370\370\370\\N*\037&$2Dlvrsieeckhgo`e6.\"/2l|",
l="Relm5-2:Oilm.+)=]j\\O==?BETUQPTXEND9+lmiYA.,1agm`HILI3),3MS=,nok`KEDJC7)6XkjcH-*3>F:Dhmkgigll*.)8BIFDA@BHXckkdfjg))+;H";
E("silverst.trk","SILVERST.TRK")
v=" ((=E?BBRP7+4&+18;>+.3-366676:7:9=6:2;;KD6>LOXf_dB\037:LHVQT;\033CbT@3JKKTPN`MI@MIROSTUF+6=37<J2\036:B@JTIYfR+\' 66:7?HF>CVWX]b",
l="KSdg+)18FI2+fih]G6-7gejcOIL>4*22?HU=</)/BJKLE714@Vfg;-+0M^YXaffK2)-6\\fbS6)+5\\dfd]age0/+-<LWNYcd[J.6+hgdS:/+/AD97ajebY";
E("spa.trk","SPA.TRK")
v="F61$\024\025:5EBRPPkvg^DIZ`itffXVTSI3L\\UXXY[h]amst{`?^urZ`N:*6769@;-89@BFGHAKCO_bV@+,8:B:GK88;IUPWPD8EOHO_ccjajW>>FCFD9\'1>86.:HA6@PSROP9,Gilrhqiw\201oppn\204aB`zxwOIG:BF;>HHL3ACO<G*+-3>XQRY",
l="Y^d`/9)*H_ddOG=6=CB95*0<_jhdULMG0)+7PcdeX@378>JJYXjagff[*)-29>JZjifTC)1:Oa]`1)+1-3+@MUPS`fdV@,7-W]hjfacVKXd_1))9Y]_`\\QSN?81370).FFA<.*-4A97@bjj]E9?>;@2;gjjQ4)*3MVID)/+8bjjeH@QYY";
E("suzuka.trk","SUZUKA.TRK")
v="22<AOGHDGC>==:0<K?JEB?IB@E8B?@HMHGDHLMLae2(-3677E86BD>BENbtdaO?27.3*1=5797(D\\]_Q98\',(#&9=?EGTXSWmmhkfE-HNPRC<;.344LIG;,JX^rmmW9LKGPQaI60.(89FC>?EVvZZ",
l="Prw\200HCEVaYSF.+,6Ylhd\202|\203kWSL9---6OL]e\200\203\203ymYB*,*)6RXZ`ey~\203|xtkY[ln?9-<59BH\\puc8-),*ADVzz\201oSH@A/))*/+40.<@Wgv\203xg[]op}zwQA:55FPH<75+o\203ypZe\177v)),Sz\203\200p[-5>N";
E("tremblnt.trk","TREMBLNT.TRK")
v="\026\':KZPRG75+6=.2959FF&\'\"%\',*6.8;8MPPPU>7B?KNSSADGe\\`cYG<:=59...9>LNQOXC.Oa`{moR/<CHHRJ@.6EN^IPIF;2Nh6\025\036)?>59;D=0?UcZUdF,%\037\031%$,-2AK7ICJEEFS\\`YocT\\kkcZVRYW]XY\\V",
l="X_b[KCNMWJ\\RG414<4018fjmokmSD*27TomfZU\\b7.).GUXanmmmacji32/))))*4536BSZaklmlkgmc;2).59GTkilaN:AG@=A<GBFRdoogK:81.1<HZdii*),,1)+.88@=BOS`jlmna[NIGA?7-)*++6@IW";
E("watglen.trk","WATGLEN.TRK")
v="<A<D;;CCI87?W\\cf`OJ]ch`lbWMSX_decM7DMFSM]O(7A8?FVHDC259RH::93(19_HS53,.<><6;NQRV]I6454;4@",
l="^_fe4)3A\\fa_C*)+BCINceicTRVS-+.7Lcbh5*-+EPXWige\\HHGA44-)Cdbh7+..9-/,cjf^C195]jjY@6GO,)/?Z";
E("zandvort.trk","ZANDVORT.TRK")
v="\370\204\204x|\204}~\177\035\037HLJLYPAGPGIKNN24HTI^V[6&Afg\201pp[JQhaedkaK]cahlfMZLFK]UT8.Rtm\217pSgjg^ph}\200\211\201ljhld\"\027#\"(/22@?/>MLCJF8AFPV^2 115*\'.C>",
l="V4/@fj\\I;??o\241\217\236\241\222\231\234\2010)*4PUwt\225\235\241\230~`Zauyvpe]bs\236\240\236\223\200vYR61*-8EYh\211\242\242\216dRSU\\]XQHG?9/-+3;Pi\201\223\237\242\237\223\236\215c+)-,O`p~\227\241\241\233~iWG)),))6?q\240\235\242\213_";
}double L=s.to_lft,R=s.to_rgt,u=s.cur_len,e=s.to_end,a=s.cur_rad,f=p*e/u,z,t,h,g
,C,D,tc=s.time_count;int I=s.seg_ID,j=f,i=p*I+p-f,H=s.laps_to_go,k,li0=0;
if(S!=I&&!I)G++;S=I;if(!l[0]||s.damage>25e3)z=L+4,r.vc=e/4*(a>0?a:1-a
)+61;else z=A(l)-36,r.vc=e/4+A(v)/.6;
if(tc<2||!s.damage&&!I&&s.vn<1e-9&&s.vn>-1e-9&&e>20&&tc<30){r.alpha=0;return r;}
for(k=3,t=9;k--;)if(N.who<16){double x=N.
rel_x,y=N.rel_y,X=N.rel_xdot,Y=N.rel_ydot;g=h=9;C=CARLEN+10+y/8;if(e>3*(L+R)&&u-
e>2*(L+R))D=1.2*CARWID;else D=(L+R)/4+y/8;if(Y<0&&y<CARLEN+10-Y*20){if((x-li<0||
L+x<5+1.2*CARWID)&&x>-2*D&&R-x>5+1.2*CARWID)li0--;else if((x-li>0||R-x<5+1.2*
CARWID)&&x<2*D&&L+x>5+1.2*CARWID)li0++;}if(x<-D){if(X>0)h=-(x+D)/X;else continue
;}else{if(x>D){if(X<0)h=-(x-D)/X;else continue;}else h=0;}if(y<C||s.v*s.v-(s.v+Y
)*(s.v+Y)>2*32*(y-CARLEN))g=0;else if(Y<0)g=-(y-C)/Y*99/(99-Y);if(h<g)h=g;if(t>h
)t=h;}if(li0<0&&(!(I&1)||e>300))li-=R/80;if(li0>0)li+=L/80;if(t<1&&(s.
damage>1e4||t<.3+.7/1e4*s.damage))r.vc=.7*s.v;if(s.nearby[0].who>15)li*=0.9;else
li*=0.97;if(li>0&&z+li>L+R-7)li=L+R-7-z;if(li<0&&z+li<7)li=7-z;z+=li;if(z-R<6&&R
-z<6)c=2;if(c<2&&!s.damage&&!I&&tc<30&&e>200)z=(z+R*9)/10;r.alpha=(z-R
)/36-s.vn/s.v;if(s.to_lft<4&&s.vn>0){if(s.to_lft<10*s.vn)r.alpha=-1;else r.alpha
-=0.1;}if(s.to_rgt<4&&s.vn<0){if(s.to_rgt<-10*s.vn)r.alpha=1;else r.alpha+=0.1;}
stuck(s.backward,s.v,s.vn,L,R,&r.alpha,&r.vc);if(s.fuel<0.003&&s.v>30&&r.vc>s.v-
2)r.vc=s.v-2;r.request_pit=0;if(s.stage!=QUALIFYING&&(s.damage>17000||s.fuel<5))
{r.request_pit=1;r.repair_amount=s.damage;r.fuel_amount=150;}return r;}
