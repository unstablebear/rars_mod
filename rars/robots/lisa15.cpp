/*
File name:      LISA15.CPP
Author:         Henning Klaskala
E-mail:         bm321465@muenchen.org
Robot name:     Lisa
Races:          all
Source code is: public
Data file:      none
Date:           11 May 1997
Very fast on:   brands,cstlcomb,brazil,buenos,imola,monaco,monza-76,mosport,
		nurburg,oval2,ra,silverst,spa,suzuka,tremblnt,watglen,zandvort.

Sponsors please send money, diamonds, gold bars etc. to this address:   ;-)
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
#define E(x,y) else if(!strcmp(T.sName,x)||!strcmp(T.sName,y))
con_vec Lisa(situation&s)
{con_vec r;static int S=0,c=0,G=0;static double li=0;static track_desc T;
if(!c){c=1;my_name_is("Lisa");r.alpha=r.vc=0;return r;}
if(s.starting){T=get_track_description();S=0;G=0;li=0;}
double L=s.to_lft,R=s.to_rgt,u=s.cur_len,e=s.to_end,
a=s.cur_rad,f=4*e/u,z,t,h,g,C,D;
int I=s.seg_ID,j=int(f),i=int(4*I+4-f),H=s.laps_to_go,k,li0=0;
char*v,*l="";
if(s.time_count<2){r.alpha=0;r.vc=9999;return r;}if(0);
E("mlwaukee.trk","MLWAUKEE.TRK")
v=".OSRIFQ_[2HJIJQSZ",
l="75,,SUZ[H8/+IY[[C";
E("indy500.trk","INDY500.TRK")
v="&C<OehmrwXXgrlmpte=Pklox|MF`nolut",
l="@/.2`kmgI512HeieD.3+`ndY6,02MiodE";
E("nazareth.trk","NAZARETH.TRK")
v="EO>MbfnkuQINXJLSaJUYPILVc",
l="<528KYYP7/30EJOXB,-1JOKVG";
E("loudon.trk","LOUDON.TRK")
v="JFHOBECCEBPVRKIQSDMNBCCCIJIKJCOSX",
l="B52.T[KCBR;FGP[[A+/0YXPGGD;FJS[[G";
E("michigan.trk","MICHIGAN.TRK")
v="\201{\222x\203\213\233\216qmzrur}tqq\205\204r\222\216\234\251",
l="QMLLOQB8N^^_L,2,VcU[VZbcR";
E("monaco.trk","MONACO.TRK")
v="&45\037\030\031\033*<86216C3*\"\036\026\030\016\025\025!*%#--$4E-(&!#\'.,:0Tdj^cgP\031\030\033\031\037\036\033.9\034\037\030\036\036*33,6,.)89,3D;=9\0260,)22/530DLIHE3\022*4>6;IC;3A>GM<D=C7CHD>9J9;MP0\037\013\015\023\0332F1+\037&&)*<9=@>Y|X (&31*8I^UU[ZM2:\0340637106F34:CMCHc",
l="MXSS*))8S\\ZWUj_PWWZijhidQOP<+)8?NRZB8+*2HVQC4+,5LY][jjaRKB?0*+194H?:.;4DGNOd^_dNJ54PjhdTH56Egfbg`VH>=,68476+)*)4>IV`hjgjNEEdhY^SB*)4Q_i=)))BfjhdVP_a))1*IC<?>?7/,---,,,6ehifUI935))3G";
E("imola.trk","IMOLA.TRK")
v="...H}ozvs\\,,2\"%&)\'-0=3)4EDOBH3\"BUQ=?>;;=B;L/\027 \0365??<TefbTN0\036,%$->@4\">RZ`]TJBC:<*MUF/543988<):XRNST>@@??9-,\027\002\022+ 5<UXUUVX[\\f[KTaA!&+,/./\'\'#)).543\'>T[^TFC601=?A?&\027%;E;?C",
l="IFGBGGKFEB33nleG))*1BMITokkbS60/kioT5+-8OXcdjoibDMdUB5300,69ejjcMGNRWVTULY_QCGFNMJUI<10<IQMZjnmi_^ce).+IfmohQ20C__bP@7:7/).273-3ono[E:8Oion^OJF?,)./>?8ZonmUFLQE/)*5@";
E("cstlcomb.trk","CSTLCOMB.TRK")
v="; \032Blokfu[(St}ga\\Q[;ZP^eH60\";008>0\";QORXRG6Mbccd_H+>9@:?<<8KNGSFF4\037?kkhmoW?:CFPOM",
l="ZmdT.)/7E_cQ50+2AEN_nooob\\J=3./6Vdjm,)+;OhkkmmieikeS5DQLEC78*06@Pmkd?7>BHflk++.>Q";
E("brands.trk","BRANDS.TRK")
v="\351\211*).!\036(0843BHFFIGLT_I/GWC:18>>BNF,?[iXgjRBLCQ]YZVHROUMVPLJPWbhd]DMWS]V[ZSFV\\``cHIMRVOPRP?)#138C\\@PBEDBJMUYcmH*LkZKQKMTS_[P[]\\^JC",
l="_heY*+/)=47?VdhjjcbYI:-1UfgZgjh^F550+-29?EbhC3+/N\\f\\=2/ALSTKC?9CEDHC746>GEJ>-)21@IFP>)*)2.,PjfjbJ@VR1)+0:B71C\\dfQJE87,.6JQYYVXXR[";
E("brazil.trk","BRAZIL.TRK")
v="0\034K0\032\037\'@,,27C:5:>15>BAG0!\033!4A-#AKDPXI/-@>89UY_N?5078N9.0(*,9D8HLOVMZC9&(,\"\0350/<L7658G]CG$%+..K15Njhmaa3D]wcSX\033",
l="TA=OpnmqC+)3HSGGfokeH+-@qolT;*.Ckqnl^\\lZ+)3Dboi`B,)*>HJM]mqk=73.*).5AVeF/)+)D<9?jpq`B..6qqpb>0?TeoqfG?CJTm]Rc";
E("oval2.trk","OVAL2.TRK")
v="847OGC;GQbHBE@PGV",
l="K<4=o\202\200\203lZOIYs~\200[";
E("buenos.trk","BUENOS.TRK")
v=",BP1\"$#(JN`PMB7=Q[]bheI+(\034!6;8/K;>HFWQHENE<9<98BBDCEVT][aU)(#%#1C\'1S_GQU]GRA;876@;:A=JLB4)/4?DC) \032\026\03595.6*24;7D:;IOMFX4@<0(1=;9#=aV__g",
l="^U^]-,)3337<V^XN52,0?XdehhgiM.+;`jeS0-.1@AH<-,)3Jafe8))3PU_B+),*EUYXY`LC0/*).:LUejfYF=-Pajhd=-26hcge`\\N9.++1MYWSGA@>EZ^L)*))@LU]fjfe_";
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
if(S!=I&&!I)G++;S=I;if(!l[0]||s.damage>24e3-14e3*H/(H+G))z=L,r.vc=e/4*(a>0?a:1-a
)+50;else z=A(l)-36,r.vc=e/4+A(v)/.6;for(k=3,t=9;k--;)if(N.who<16){double x=N.
rel_x,y=N.rel_y,X=N.rel_xdot,Y=N.rel_ydot;g=h=9;C=CARLEN+10+y/8;if(e>3*(L+R)&&u-
e>2*(L+R))D=1.2*CARWID;else D=(L+R)/4+y/8;if(Y<0&&y<CARLEN+10-Y*20){if((x-li<0||
L+x<5+1.2*CARWID)&&x>-2*D&&R-x>5+1.2*CARWID)li0--;else if((x-li>0||R-x<5+1.2*
CARWID)&&x<2*D&&L+x>5+1.2*CARWID)li0++;}if(x<-D){if(X>0)h=-(x+D)/X;else continue
;}else{if(x>D){if(X<0)h=-(x-D)/X;else continue;}else h=0;}if(y<C)g=0;else if(Y<0
)g=-(y-C)/Y*99/(99-Y);if(h<g)h=g;if(t>h)t=h;}
int dpL=T.NSEG==20&&(I==1||I==2&&e>600||I==7||I==8&&e<400||I==9||I==10||I==19);
dpL|=T.NSEG==33&&(I==1||I==2||I==7&&e<200||I==11||I==13||I==30);
dpL|=T.NSEG==32&&(I==11||I==13||I==14||I==18||I==19||I==20);
dpL|=T.NSEG==41&&(I==14||I==13||I==19||I==20||I==8||I==25||I==24&&e<200||I==7&&e<200||I==18&&e<200);
dpL|=T.NSEG==45&&(I==7||I==8||I==34||I==33&&e*a>-300);
int dpR=T.NSEG==33&&(I==0&&e<500||I==7||I==8||I==9||I==19||I==20||I==21);
dpR|=T.NSEG==20&&(I==0&&e<200||I==6||I==8&&e<400);
dpR|=T.NSEG==32&&(I==23||I==18);
dpR|=T.NSEG==41&&(I==29||I==28||I==26||I==8||I==9||I==32||I==33||I==7&&e<200);
if(li0<0&&!dpR)li-=R/80;if(li0>0&&!dpL)li+=L/80;if(t<1&&(s.
damage>5e3||t<.3+.7/5e3*s.damage))r.vc=.7*s.v;if(s.nearby[0].who>15)li*=0.9;else
li*=0.97;if(li>0&&z+li>L+R-7)li=L+R-7-z;if(li<0&&z+li<7)li=7-z;z+=li;if(z-R<6&&R
-z<6)c=2;if(c<2&&(!s.damage||s.nearby[0].who>=16)&&!I&&s.time_count<50&&e>200)z=(z+R*9)/10;r.alpha=(z-R
)/36-s.vn/s.v;if(s.to_lft<4&&s.vn>0){if(s.to_lft<10*s.vn)r.alpha=-1;else r.alpha
-=0.1;}if(s.to_rgt<4&&s.vn<0){if(s.to_rgt<-10*s.vn)r.alpha=1;else r.alpha+=0.1;}
stuck(s.backward,s.v,s.vn,L,R,&r.alpha,&r.vc);if(s.fuel<0.003&&s.v>30&&r.vc>s.v-
2)r.vc=s.v-2;
if(s.starting) r.fuel_amount = MAX_FUEL;
r.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
  r.request_pit = 1;
  r.repair_amount= s.damage;
  r.fuel_amount = MAX_FUEL;
  }
return r;
}
