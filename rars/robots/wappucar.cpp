// This is robot driver 'WappuCar' by Jussi Pajala, May, 1995

// This is slightly modified version of Wappucar which won the second
// place in april 30 races. I have already done some translation to
// english ( only most important variable and function names).
// There is also some english comments.

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "track.h"

#define false 0
#define true 1
#define pii 3.14159265359

// curve types
#define straight 0    // straight segment
#define norm_curv 1 // normal curve, straight before and after it
#define first_curv 2 // straight before and curve to same direction after it
#define last_curv 3 // last curve of this direction, straight after it
#define mid_curv 4 // in the middle of the curves of same direction
#define norm_shic 5 // shicane; curve to the other direction before and
		    // straight or shicane after it
#define last_curv_shic 6 // some curves of the other direction before and
			 // straight or shicane after it
#define shic_first_curv 7 // a curve of the other before and some
			  // curves of same direction after it
#define last_curv_shic_first_curv 8 // some curves of the other direction
				    // before and some of the other direction
				    // after

extern const double CARWID;

//extern char trackfile[];
extern int no_display;

const char name[] = "WappuCar";

// parameters for each track
const char tracks[8][16]={ "zandvort.trk","oval2.trk","stef2.trk","speed2.trk",
			   "v03.trk","anew.trk","fourmile.trk","4mile"};
const int known_tracks=8;
const double consts[8][10]={{31,25,5.5 , .15,.25,.15,.7,.2, 50, 5},// zandvort
			    {31,25,5.9 , .15,.25,.15,.8,.2, 35,.5},// oval2
			    {34,30,5.65, .15,.25,.15, 0,.1, 55, 5},// stef2
			    {33,25,5.6 , .15,.25,.15,.8,.2, 35, 1},// speed2
			    {31,25,5.65, .15,.25,.15, 1,.2, 20, 0},// v03
			    {30,30,5.62, .15,.34,.23,.8,.15,27,12},// anew
			    {31,20,5.75, .15,.25,.15,.7,.2 ,65, 0},// fourmile
			    {31,20,5.75, .15,.25,.15,.7,.2 ,65, 0}};// 4mile

struct ratataul {
  int tyyppi;    //curve type
  double ajor;   //driving lines rad
  double r;      // curves rad
  double pituus; //length
};


class WappuCar : public Driver
{
public:

  int NSEG;              // number of segments
  segment *rgtwall;     // track defining arrays - see TRACK.CPP
  segment *lftwall;

  // parameters :
  double a;      // acceleration on straight
  double kurv_a; // acceleration on curve
  double v_kerr;// speed rate
  double accslad,decslad;// max acceleration and braking slide percent
  double curvslad; // max slide percent on curve
  double ajolinjakerr;// braking rate when not on driving line
  double maxkulma;    // max angle to drive to the line
  double vara;        // space to outer curve
  double sisavara;    // space to inner curve

  // v_kerr, vara and sisavara are the most important parameters

  double vc,alpha;
  int init_flag;

  con_vec result;
  double r1,r2;              // curves inner and outer radian ( with space )
  double nex_r1,nex_r2;      // and the same for next curve
  double r3,nex_r3;          // driving lines radian
  double nex_yli;            // how far the driving line is from end of straight
  double v0,x;               // speed on next curve, x=distance to curve
  int ulkona;   // if out of road during last round
  int tracknum;     // current tracks number
  int n;              // current segments number
  int ohitus;       // <>0 when passing if <0 passing from right side
  int firstround;// true when called first time
  char trackname[16];
  ratataul *curves;

  WappuCar()
  {
    a=31;      // acceleration on straight
    kurv_a=25; // acceleration on curve
    v_kerr=5.6;// speed rate
    accslad=.15;
    decslad=.25;// max acceleration and braking slide percent
    curvslad=.15; // max slide percent on curve
    ajolinjakerr=.7;// braking rate when not on driving line
    maxkulma=.2;    // max angle to drive to the line
    vara=35;        // space to outer curve
    sisavara=7;    // space to inner curve

    vc=1000;
    alpha=0;
    tracknum = 0;
    ohitus = 0;

    firstround = true;
    ulkona = false;
    init_flag = true;

    m_iNoseColor = oWHITE;
    m_iTailColor = oWHITE;
    m_sBitmapName2D = "car_white_white";
    m_sModel3D = "futura";
    m_sName = "WappuCar";
  }

  double wc_driving_lines_rad(int tyyppi,double r,double omega);
  void wc_track_init(void);
  double wc_abs(double x);
  double wc_sqr(double x);
  double wc_minimum(double a,double b);
  double wc_laske_yli(int tyyppi,double r1,double r2,double r3,double omega);
  void wc_laskeperusjutut(situation s);
  void wc_calc_driving_line(situation s);
  void wc_speed(situation s);
  double wc_anna_fii(int tyyppi,double pit,double loppuun);
  void wc_passing(situation s);
  void wc_sladiraj(double nykyv);
  void wc_alpha(situation s);
  double wc_anna_theta(situation s);
  
  con_vec drive(situation& s);

};

/*void piirramit(double v){
#define xx 250
#define yy 50
#define mit_r 40
static int mx=xx,my=yy;
  setcolor(2);
  line(xx,yy,mx,my);
  my=yy+mit_r*cos(v*.021);
  mx=xx-mit_r*sin(v*.021);
  setcolor(0);
  line(xx,yy,mx,my);
} */

double WappuCar::wc_driving_lines_rad(int tyyppi,double r,double omega){
double r1,r2=0;

  // outer and inner curve with spaces
  if (r<0){ r1=r-sisavara; r2=r-get_track_description().width+vara;}
  else { r1=r+sisavara; r2=r+get_track_description().width-vara; }

  // driving lines radian
  switch(tyyppi){

  case norm_curv: return (r2-r1*cos(omega/2))/(1-cos(omega/2));// kaava 1;
  case norm_shic: return (r2-r1*cos(omega/2))/(1-cos(omega/2));

  case first_curv:                return (r2-r1*cos(omega))/(1-cos(omega));
  case shic_first_curv:           return (r2-r1*cos(omega))/(1-cos(omega));
  case last_curv_shic_first_curv: return (r2-r1*cos(omega))/(1-cos(omega));

  case last_curv:      return (r2-r1*cos(omega))/(1-cos(omega));
  case last_curv_shic: return (r2-r1*cos(omega))/(1-cos(omega));

  case mid_curv:return r1;
  default:return 0;
  }
}

void WappuCar::wc_track_init(void){
int i=0;

NSEG = get_track_description().NSEG;
lftwall =  get_track_description().lftwall;
rgtwall =  get_track_description().rgtwall;

  firstround=false;n=0;
  strcpy(trackname,get_track_description().sName);       // vain 1. kierroksella

  for(tracknum=0; tracknum<known_tracks; tracknum++)
    if (strcmp(tracks[tracknum],trackname)==0) break;
  if (tracknum<known_tracks){
    a=consts[tracknum][0];
    kurv_a=consts[tracknum][1];
    v_kerr=consts[tracknum][2];
    accslad=consts[tracknum][3];
    decslad=consts[tracknum][4];
    curvslad=consts[tracknum][5];
    ajolinjakerr=consts[tracknum][6];
    maxkulma=consts[tracknum][7];
    vara=consts[tracknum][8];
    sisavara=consts[tracknum][9];
  }


  curves=new ratataul[NSEG+3];

  for (i=0;i<NSEG;i++){  // count inner radian of the curve:
    if (lftwall[i].radius==0&&rgtwall[i].radius==0) curves[i].r=0;
    else if (lftwall[i].radius<0) curves[i].r=rgtwall[i].radius;
    else curves[i].r=lftwall[i].radius;
    curves[i].pituus=lftwall[i].length;
  }
  curves[NSEG]=curves[0]; // copy first radian to last

  // identify curve types:
  for(i=0;i<NSEG;i++){
    if (curves[i].r==0) curves[i].tyyppi=straight;

    else if (curves[i].r*curves[i-1].r>0) // if curve before is to same direction
      if (curves[i].r*curves[i+1].r>0) curves[i].tyyppi = mid_curv;
      else if (curves[i].r*curves[i+1].r<0)
	curves[i].tyyppi = last_curv_shic;
      else curves[i].tyyppi = last_curv;

    else if (curves[i].r*curves[i-1].r==0)  // if segment before is straight
      if (curves[i].r*curves[i+1].r>0) curves[i].tyyppi = first_curv;//next is to same direction
      else if (curves[i].r*curves[i+1].r<0) //if next is to the other direction
	curves[i].tyyppi = norm_shic;
      else curves[i].tyyppi = norm_curv; //if next is straight too

    else  // if curve before is to the other direction
      if (curves[i].r*curves[i+1].r>0) //if next is to same dir
	if (curves[i+1].r*curves[i+2].r>0) // and next one too
	  if (curves[i-1].tyyppi==last_curv_shic)
	     //if curve before was the last of the same direction
	    curves[i].tyyppi=last_curv_shic_first_curv;
	  else curves[i].tyyppi=shic_first_curv;//if curve before is norm_curv or shicane
	else curves[i].tyyppi = shic_first_curv;
      else if (curves[i].r*curves[i+1].r<0)
	curves[i].tyyppi = norm_shic; //curve before and after to the other direction
      else curves[i].tyyppi = norm_shic; //curve before to other direction,
					 // straight after
  }

  // driving lines radian
  for(i=0;i<NSEG;i++){
    switch(curves[i].tyyppi){
      case straight: curves[i].ajor=0; break;
      default: curves[i].ajor
	       = wc_driving_lines_rad(curves[i].tyyppi,curves[i].r,lftwall[i].length);
    }
  }
  curves[NSEG]=curves[0];   // copy track from the beginning to the end
  curves[NSEG+1]=curves[1];
  curves[NSEG+2]=curves[2];


}


double WappuCar::wc_abs(double x){
  if (x<0) return -x; else return x;
}

double WappuCar::wc_sqr(double x){
  return x*x;
}

double WappuCar::wc_minimum(double a,double b){
  if (a<b) return a;
  else return b;
}


double WappuCar::wc_laske_yli(int tyyppi,double r1,double r2,double r3,double omega){
// this routine calculates how far on the straight the driving line begins
  double d;
  switch(tyyppi){
    case norm_curv:
    case norm_shic:
      d=sqrt(wc_sqr(r2)+wc_sqr(r3-r1)-2*r2*(r3-r1)*cos(pii-omega/2));// kaava 2
      break;
    case first_curv:
    case last_curv:
    case last_curv_shic:
    case shic_first_curv:
    case last_curv_shic_first_curv:
      d=sqrt(r2*r2+(r3-r1)*(r3-r1)-2*r2*(r3-r1)*cos(3.142-omega));// kaava 2
      break;
    default:d=r3;
  }
  return sqrt(d*d-r3*r3);  // kaava 3
}


void WappuCar::wc_laskeperusjutut(situation s){

  r1=r2=nex_r1=nex_r2=0;  // nollataan s„teet

  // lasketaan kurvin sis„- ja ulkos„de
  if (s.cur_rad<0){
	  r2=s.cur_rad-get_track_description().width+vara;
	  r1=s.cur_rad-sisavara;
  }
  else if (s.cur_rad>0){
	r2=s.cur_rad+get_track_description().width-vara;
	r1=s.cur_rad+sisavara;
  }
  // samoin sis„- ja ulkos„de seuraavalle kurville
  if (s.nex_rad<0){
	  nex_r2=s.nex_rad-get_track_description().width+vara;
	  nex_r1=s.nex_rad-sisavara;
  }
  else if (s.nex_rad>0){
	nex_r2=s.nex_rad+get_track_description().width-vara;
	nex_r1=s.nex_rad+sisavara;
  }
}


void WappuCar::wc_calc_driving_line(situation s){  // lasketaan ajolinjan s„de ja ylitys, jos mahd.
  if (s.cur_rad==0) r3=0;
  else r3=curves[n].ajor;

  if (s.nex_rad==0){ nex_r3=0;nex_yli=0;}
  else {
	nex_r3=curves[n+1].ajor; // ajor = driving lines radian
	nex_yli=wc_laske_yli(curves[n+1].tyyppi,nex_r1,nex_r2,nex_r3,s.nex_len);
	    // see wc_laske_yli function
  }
}


void WappuCar::wc_speed(situation s){

  vc=1000;
  // nopeus suoralla, ja mutkaan tullessa
  if (r3==0){
	if (nex_r3!=0){
	   x=s.to_end-nex_yli;
       if (x<0) x=0;
	   v0=v_kerr*sqrt(wc_abs(nex_r3));
	   wc_minimum(vc=sqrt(v0*v0+2*a*x),vc);

       if (wc_abs(curves[n+2].ajor)<wc_abs(curves[n+1].ajor)&&curves[n+2].r!=0){
	 x=s.to_end+wc_abs(s.nex_rad*s.nex_len); // et„isyys mutkan alkuun
	     v0=v_kerr*sqrt(wc_abs(curves[n+2].ajor));
	     vc=wc_minimum(sqrt(v0*v0+2*kurv_a*x),vc);
       }
       // tai sit„ seuraava
       if (wc_abs(curves[n+3].ajor)<wc_abs(curves[n+1].ajor)&&curves[n+3].r!=0){
	 x=s.to_end+wc_abs(s.nex_rad*s.nex_len); // et„isyys seur. mutkan
				      // takaisen palikan alkuun
	 if (s.after_rad!=0) x+=wc_abs(s.after_rad)*curves[n+2].pituus;
	 else x+=curves[n+2].pituus;

	     v0=v_kerr*sqrt(wc_abs(curves[n+3].ajor));
	     vc=wc_minimum(sqrt(v0*v0+2*kurv_a*x),vc);
       }
	}
  }
  else { // nopeus mutkassa, tiukempaan mutkaan tullessa
    if ((wc_abs(nex_r3)<wc_abs(r3)&&wc_abs(nex_r3)!=0)){ //jos seuraava kurvi tiukempi
      x=s.to_end*wc_abs(s.cur_rad)-nex_yli;  // vois ehk„ v„h„n tarkentaa, et„lopp ehk„ v„h„n
			  // turhan pitk„ ( s.cur_rad*s.to_end )
      if (x<0) x=0;
      v0=v_kerr*sqrt(wc_abs(nex_r3));
	  vc=sqrt(v0*v0+2*kurv_a*x);
    }
    else if(wc_abs(curves[n+2].ajor)<wc_abs(r3)&&curves[n+2].r!=0){//nopeus mutkassa,
				      // kun seuraavan seuraava kurva tiukempi
      x=s.to_end*wc_abs(s.cur_rad);

      if (s.nex_rad!=0)  x+=wc_abs(s.nex_rad)*s.nex_len;
      else x+=s.nex_len;

      v0=v_kerr*sqrt(wc_abs(s.after_rad)+get_track_description().width);
      vc=sqrt(wc_sqr(v0)+2*kurv_a*x);
    }
    vc=wc_minimum( vc,v_kerr*sqrt(wc_abs(r3)) );  // nopeus mutkassa
  }

}
double WappuCar::wc_anna_fii(int tyyppi,double pit,double loppuun){
double fii=0;
  // fii on "autosta kurvin keskipisteeseen ja kurvin keskipisteest„
  // ajos„teen keskipisteeseen" v„linen kulma

  switch(tyyppi){
    case first_curv                : fii = loppuun;break;
    case shic_first_curv           : fii = loppuun;break;
    case last_curv_shic_first_curv : fii = loppuun;break;

    case last_curv      : fii = pit-loppuun;break;
    case last_curv_shic : fii = pit-loppuun;break;


    case norm_curv : fii = 0.5*pit-loppuun;break;
    case norm_shic : fii = 0.5*pit-loppuun;break;
    case mid_curv : fii = 0.5*pit-loppuun;break;

    default        : fii = 0;
  }
  if (fii>0) fii=pii-fii;
  else fii=pii+fii;
  return fii;
}

double WappuCar::wc_anna_theta(situation s){
  // theta on kurvin suunnan ja ajoympyr„n tangentin v„linen kulma

  double A,B,C,D,E;
  double alfa,beta,epsilon,zeta,fii,theta;
  double v0,x,y; // v0 ja x nopeuden laskemisessa, x ja y peilattaessa
		 // shikaania suoralle

  if (s.cur_rad!=0){
  //Jos ollaan kurvissa suoritetaan t„m„ :
    A=r3-r1;if (A<0) A=-A;

    if (s.cur_rad>0)B=s.cur_rad+s.to_lft; // B = et. kurvin keskustaan
    else B=-s.cur_rad+s.to_rgt;

    fii=wc_anna_fii(curves[n].tyyppi,s.cur_len,s.to_end);

    C=sqrt(wc_sqr(A)+wc_sqr(B)-2*A*B*cos(fii));

    // Jos ollaan ajolinjan sis„puolella palautetaan nolla
    if (C<r3 || ( r3<0&&C<-r3)) epsilon=0;

    // Jos ollaan ajolinjan ulkopuolella jatketaan laskemista
    else {
      if (curves[n].tyyppi!=mid_curv){

      //jos tyyppi = 'mid_curv'  ei tarvita n„it„:
	zeta=acos( (wc_sqr(B)-wc_sqr(C)-wc_sqr(A))/(-2*A*C) ); // zeta = kulma(A,C)

	alfa=r3/C;if (alfa<0) alfa=-alfa;
	alfa=acos(alfa);if (alfa<0) alfa=-alfa; // alfa = kulma(C,r3)

	if (s.to_end>.5*s.cur_len&&
		(s.cur_len-s.to_end)+zeta<.5*s.cur_len)
	  beta=zeta-alfa;
	else if (s.to_end>.5*s.cur_len) beta=alfa-zeta;
	else beta=alfa+zeta;// alfa+zeta

	if (r3>0) E=sqrt(wc_sqr(r3)+wc_sqr(A)-2*A*r3*cos(beta));
	else E=sqrt(wc_sqr(r3)+wc_sqr(A)-2*A*(-r3)*cos(beta));
      } else  E=r3;

      D=sqrt(wc_sqr(C)-wc_sqr(r3));
      epsilon=acos( (wc_sqr(E)-wc_sqr(D)-wc_sqr(B))/(-2*D*B) );

      epsilon=.5*pii-epsilon; // oikeastaan t„m„ on jo theta
    }
    // Jos shikaanissa, lasketaan tangentti seuraavalle segmentille
    if ((s.to_end<.5*pii)&&(curves[n].tyyppi==norm_shic||
//        curves[n].tyyppi==shic_first_curv||
	curves[n].tyyppi==last_curv_shic//||
//        curves[n].tyyppi==last_curv_shic_first_curv)
      )){

      if (s.cur_rad<0) D=-s.cur_rad+s.to_rgt;
      else D=s.cur_rad+s.to_lft;

      y=wc_abs(r1)-D*cos(s.to_end);
      x=D*sin(s.to_end);
      if (x<nex_yli){ // jos ajolinja mahdollista laskea

	C=nex_yli-x;
	alfa=atan(C/wc_abs(nex_r3));

	if (nex_r3>0){
	  B=nex_r3-sqrt(wc_sqr(nex_r3)-wc_sqr(C));
	  A=nex_r3+(B+y)*cos(alfa);
	}
	else {
	  B=-nex_r3-sqrt(wc_sqr(nex_r3)-wc_sqr(C));
	  A=nex_r3-(B+y)*cos(alfa);
	}

	// jos ollaan ajolinjan sis„puolella
	if (wc_abs(A)<wc_abs(nex_r3)) {
	   // Jos ollaan ajolinjan sis„puolella, v„hennet„„n nopeutta
	  if (ajolinjakerr!=0){ // jos nopeudenv„hennyst„ ei ole estetty
	    x=(s.to_end-nex_yli); // et„isyys "ajolinjan" alkuun
	    if (x<0) x=0;
		v0=ajolinjakerr*wc_sqr(wc_sqr(A/nex_r3))*v_kerr*sqrt(wc_abs(nex_r3));
		vc=sqrt(v0*v0+2*a*x);
	  }
	  return epsilon ;
	}
	beta=asin(nex_r3/A); if (beta<0) beta=-beta;
	theta =  s.to_end - (.5*pii+alfa-beta);

	if (s.cur_rad>0)B=s.cur_rad+s.to_lft; // B = et. kurvin keskustaan
	else B=-s.cur_rad+s.to_rgt;

	if (wc_abs(r1)<B) alfa=pii/2- asin(wc_abs(r1)/B);
	else alfa=0;// tien keskiviivan ja sis„kurvin tangentin v„linen kulma


	if ( ( wc_abs(theta)<wc_abs(alfa) || alfa*theta<0 )
/*             && ( (r3<0&&theta<epsilon) || (r3>=0&&theta>epsilon) )*/ )
	  return theta;
//        else if (r3>=0&&theta>epsilon) return theta;
      }
    }

    return epsilon ;
  }

  // Jos ollaan suoralla suoritetaan seuraava:
  else
  {
    C=nex_yli-s.to_end;
    alfa=atan(C/nex_r3);if (alfa<0) alfa=-alfa;

    if (nex_r3>0){
      B=nex_r3-sqrt(wc_sqr(nex_r3)-wc_sqr(C));
      A=nex_r3+(B-s.to_rgt+vara)*cos(alfa);
    }
    else {
      B=-nex_r3-sqrt(wc_sqr(nex_r3)-wc_sqr(C));
      A=nex_r3-(B-s.to_lft+vara)*cos(alfa);
    }


    if ( (A<nex_r3&&nex_r3>0)||(-A<-nex_r3&&nex_r3<0)){
      // Jos ollaan ajolinjan sis„puolella, v„hennet„„n nopeutta
     if (ajolinjakerr!=0){ // jos nopeudenv„hennyst„ ei ole estetty
       x=(s.to_end-nex_yli); // et„isyys "ajolinjan" alkuun
       if (x<0) x=0;
	   v0=ajolinjakerr* /*wc_sqr*/ (wc_sqr(A/nex_r3))*v_kerr*sqrt(wc_abs(nex_r3));
	   vc=sqrt(v0*v0+2*a*x);
     }

     return 0; // ja ajetaan suoraan
    }
    else {
      beta=asin(nex_r3/A); if (beta<0&&nex_r3>0) beta=-beta;
      if (nex_r3>0) return .5*pii+alfa-beta;
      else if (nex_r3<0) return -(.5*pii+alfa-beta);
    }
  }
  return 999;
}


void WappuCar::wc_alpha(situation s){  // K„„nn”ksenlaskufunktio
double v_kulma,tan_kulma;
double et=0,lev;
double suuntero=0.0;

  if (s.cur_rad!=0){  // Lasketaan keulan suunta mutkassa
    v_kulma=asin( s.vn/s.v );
    tan_kulma=wc_anna_theta(s);
    if (r3>0) suuntero=-v_kulma+/*kaantokerr**/tan_kulma;
    else if (r3<0) suuntero=-v_kulma-/*kaantokerr**/tan_kulma;
  }
  if (s.cur_rad==0){  // Ja keulan suunta suoralla
    if (s.to_end!=0&&s.to_end>nex_yli){ // kun kaukana kurvista...

      et=s.to_end-nex_yli;
      if (s.nex_rad<0) lev=wc_abs(-s.to_lft+vara);
      else lev=wc_abs(s.to_rgt-vara);
      if ( wc_abs(lev/et)>maxkulma && maxkulma!=0)

      et=s.to_end;

      if (s.nex_rad>0)
	suuntero=-atan((s.to_rgt-vara)/et)-asin(s.vn/s.v);
      else if (s.nex_rad<0)
	suuntero=-atan((-s.to_lft+vara)/et)-asin(s.vn/s.v);
      else suuntero=-asin(s.vn/s.v);

      if (suuntero<-maxkulma&&s.nex_rad>0) suuntero=-maxkulma;
      else if (suuntero>maxkulma&&s.nex_rad<0) suuntero=maxkulma;
    }
    else if (s.to_end<nex_yli){ // ... ja l„hell„ kurvia
      v_kulma=asin( s.vn/s.v );
      tan_kulma=wc_anna_theta(s);
      suuntero=tan_kulma- v_kulma;

    }
  }
  alpha=suuntero;
}

void WappuCar::wc_sladiraj(double nykyv){
  if (r3==0){        // Rajoitetaan sladia suoralla
    if (vc/nykyv>(1+accslad)){
      vc=nykyv*(1+accslad);
      if (vc<nykyv+12) vc=nykyv+12;
    }
    else if (vc/nykyv<(1-decslad))vc=nykyv*(1-decslad);
  }
  else{              //raj. sladia mutkassa
    if (vc/nykyv>(1+curvslad)&&nykyv>30){
      if (vc/nykyv<1.3)
      vc=nykyv*(1+curvslad);
    }
    else if (vc/nykyv<(1-curvslad)&&r3!=0) {
      vc=nykyv*(1-curvslad);
    }
  }
}

void WappuCar::wc_passing(situation s){
int i,vaisto_suunta=-9999;
double v,d;
double d_alfa,v_alfa;
double x,y,vx,vy;

  if (ohitus==0){
    for(i=0;i<3;i++) if (s.nearby[i].who<16){
      x=s.nearby[i].rel_x;
      y=s.nearby[i].rel_y;
      vx=s.nearby[i].rel_xdot;
      vy=s.nearby[i].rel_ydot;
      d=sqrt(wc_sqr(x)+wc_sqr(y));
      v=sqrt(wc_sqr(vx)+wc_sqr(vy));

      if (y!=0) d_alfa=atan(x/y);else d_alfa=x/(.01+wc_abs(x))*pii/2;
      if (vy!=0)v_alfa=atan(vx/vy);else v_alfa=x/(.01+wc_abs(x))*pii/2;

      if(  (v/(d+.01)>.7 || d<CARWID )
	   && vy<0
	   && d_alfa-atan(2*CARWID / (d+.001)) <v_alfa
	   && d_alfa+atan(2*CARWID / (d+.001)) >v_alfa
	   && v/s.v<.9) {

	if (s.cur_rad==0&&s.to_end>nex_yli){ // jos suoralla, v„istet„„n
	  if (vaisto_suunta<-900) vaisto_suunta=0;
	  if (s.nex_rad<0) // k„„nnyt„n mieluummin vasemmalle
	    if (s.to_lft<15||
	       (s.to_end-nex_yli>300 && d_alfa<0) ) {
	      vaisto_suunta--;// jos auto vasemmalla, v„ist” oikealle
	    }
	    else { vaisto_suunta++; /*break;*/ }
	  else
	    if (s.to_rgt<15 ||
	       (s.to_end-nex_yli>300 && d_alfa>0) ){
	      vaisto_suunta++;
	    }
	    else { vaisto_suunta--;/* break;*/ }
	  }
	else // jos ollaan mutkassa, jarrutetaan
	  vc=wc_minimum(s.v-cos(v_alfa)*sqrt(wc_sqr(vx)+wc_sqr(vy)),vc);
      }
      if (i==2&&vaisto_suunta>-800) {
	if (vaisto_suunta<0) { ohitus=-1;alpha-=.2;}
	else if (vaisto_suunta>0) {ohitus=1;alpha+=.2;}
	else if ( (s.nex_rad<0&&s.to_lft>15)
	       || (s.nex_rad>0&&s.to_rgt<15) ) { alpha+=.2; ohitus=1; }
	else { alpha-=.2;ohitus=-1; }
      }
    }
  }
  else if (ohitus!=0){
    if ( s.cur_rad!=0||s.to_end<nex_yli
	 || ohitus<0&&s.to_rgt<5
	 || ohitus>0&&s.to_lft<5 )
      ohitus=0;

    if (ohitus<0){
      ohitus--;
      if (alpha>-.2) alpha-=.2;
    }
    else if (ohitus>0){
      ohitus++;
      if (alpha<.2) alpha+=.2;
    }
    if (wc_abs(ohitus)>=15) ohitus=0;
  }
}



con_vec WappuCar::drive(situation& s)// * * * * * *  PŽŽOHJELMA * * *  * *  * * * *  * *
{
  if (init_flag){                 // kerrotaan robotin nimi,
    my_name_is(name);       // vain 1. kierroksella
    r3=nex_r3;
    result.vc=80;result.alpha=0;
    init_flag=false;
    return result;
  }
  if (!init_flag) result.fuel_amount = MAX_FUEL;
  if (!init_flag) wc_track_init(); // lasketaan radan ominaisuuksia

  // varmistutaan ett„ tiedet„„n miss„ ollaan menossa
  while (s.cur_rad!=curves[n].r||s.nex_rad!=curves[(n+1>NSEG?0:n+1)].r){
    n++;
    if (n>=NSEG) n=0;
  }

//  if (!no_display) piirramit(s.v);

  // lasketaan esim. r1,r2, nex_samat, kulmlopp, nykyv
  wc_laskeperusjutut(s);

  // seuraavassa lasketaan ajolinjojen s„teet ja ylitys, jos mahdollista
  wc_calc_driving_line(s); // l. r3, nex_r3, yli ja nex_yli

  // lasketaan uusi "velocity commanded"
  wc_speed(s);

  // uusi alpha
  wc_alpha(s);

  // v„istet„„n t”rm„yskurssilla olevia autoja
  wc_passing(s);

  // rajoitetaan renkaitten suttaamista ja lukittumista
  // prevent tire locking and sliding
  wc_sladiraj(s.v);

  // estet„„n ulosmenoa ja tullaan takaisin tielle jos ollaan ulkona
  // this is my own stuck() function
  if (s.to_lft<0||s.to_rgt<0){
    if (s.v<40) vc=s.v+20;else vc=s.v;
    if (s.to_lft<0&&s.vn>-8) alpha=-.3;
    if (s.to_rgt<0&&s.vn<8) alpha=.3;
  }
  if ((s.backward)||(ulkona)){
    ulkona=true;
    if (s.to_rgt<0||s.to_lft<0) alpha=0;
    else{
      if (s.vn<0) alpha=.5;
      else alpha=-.5;
      if (s.vn/s.v<.5&&s.vn/s.v>-.5) ulkona=false;
    }
  }

  // palautetaan lasketut arvot
  result.vc = vc;//vc*(vc<102.7)+ (vc>102.7)*102.7;
  result.alpha = alpha;
  result.request_pit = 0;
 if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
    result.request_pit = 1;
    result.repair_amount=s.damage;
    result.fuel_amount = 150;
  }
	
 return result;
}

Driver * getWappuCarInstance()
{
  return new WappuCar();
}


