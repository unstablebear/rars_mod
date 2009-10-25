// File name      : vector.cpp
// Data file name : Vector.dat
// Function name  : vector
//
// This is robot driver 'Vector' by Jussi Pajala, jussip@stekt.oulu.fi
//
// April   1996  First version ready
// January 1997  Minor changes to passing code
// March   1998  Changed data file to work with Rars v.0.71
//               Added support for different qualifying data
//
// This is quite fast driver but it has some problems. Passing is not even
// close to Wappucar3's level, mainly because driving lines are not tied to
// the road, but to global coordinates. This is also the reason why this
// version does not work with older track files: driving lines are way out
// of their right places...
//
// Don't be surprized if you see this driver driving out of the road and 
// staying there: it is also problem with driving lines. If some other
// driver pushes Vector too much away from its proper line, he may not be
// able to change driving line to the next and that's why he goes crazy.
//
// Sorry about messy code, even I'm quite confused with it. If you try to
// work out how this driver works, you are spending your time... You'll
// never get it cleared. :)
// But if you like to take a look through the code, go ahead and do it.
// If you somehow understand what this code does, and if you want to use
// it please first ask me.
//
// NOTE: This driver with datafile 'vecto071.dat' works properly only with
// Rars 0.71 and (maybe) newer.
//


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "track.h"
#include "car.h"
#include <fstream.h>
#include <iostream.h>
#define false 0
#define true 1

#define PII   3.14159265359
#define PII_2 1.57079632679


#define min(x,y) (((x)>(y))?(y):(x))
#define max(x,y) (((x)<(y))?(y):(x))
#define ulkokurviin ( (s.cur_rad==0?s.nex_rad:s.cur_rad)>0?s.to_rgt:s.to_lft)
#define sisakurviin ( (s.cur_rad==0?s.nex_rad:s.cur_rad)>0?s.to_lft:s.to_rgt)


/******************* VEKTORIMŽŽRITTELYT ***********************************/

class vector {
  double x;     // x-akselin suuntainen komponentti
  double y;     // y-akselin suuntainen komponentti
  double l;     // vektorin pituus
public:
  vector();                     // constructor
  void set(double,double);   // asettaa x:n ja y:n ja laskee l:n
  void turn(double);       //
  double get_x();            // antaa x:n
  double get_y();            // antaa y:n
  double get_l();
};

vector operator *(double r,vector v) // vektorin kertominen reaaliluvulla
{
  vector u;
  u.set(v.get_x()*r,v.get_y()*r);
  return u;
}
vector operator /(vector v,double r) // vektorin jakaminen reaaliluvulla
{
  vector u;
  u.set(v.get_x()/r,v.get_y()/r);
  return u;
}
vector operator +(vector v,vector w) // pluslasku
{
  vector u;
  u.set(v.get_x()+w.get_x(),v.get_y()+w.get_y());
  return u;
}
vector operator -(vector v,vector w) // miinuslasku
{
  vector u;
  u.set(v.get_x()-w.get_x(),v.get_y()-w.get_y());
  return u;
}
vector operator -(vector v) // k„„nt„„ vektorin suunnan
{
  vector u;
  u.set(-v.get_x(),-v.get_y());
  return u;
}
vector operator +(vector v) // pluslasku
{
  return v;
}

double operator *(vector v,vector w) // ristitulo
{
  return v.get_x()*w.get_y()-w.get_x()*v.get_y();
}

double operator ,(vector v,vector w) // pistetulo
{
  return v.get_x()*w.get_x()+v.get_y()*w.get_y();
}

/********************* muut m„„rittelyt *********************************/
enum CURVE_TYPES { straight,norm_curv,first_curv,last_curv,mid_curv,norm_shic,
                   last_curv_shic,shic_first_curv };



// track variables
static int NSEG;              // number of segments
static segment *rgtwall;     // track defining arrays - see TRACK.CPP
static segment *lftwall;
static double width;


static const char name[] = "Vector";

// parameters :
static double a=35;      // acceleration on straight
static double v_kerr=5.6;// speed rate
static double accslad=.15,decslad=.25;// max acceleration and braking slide percent
static double curvslad=.15; // max slide percent on curve
static double ajolinjakerr=.7;// braking rate when not on driving line
static double maxkulma=.1;    // max angle to drive to the line
static double vara=40;         // space to outer curve
static double norm_sisavara=.5,sisavara=.5;    // space to inner curve
// v_kerr, vara and sisavara are the most important parameters


// other variables
static double vc=1000,alpha=0;
static int init_flag=true;

static con_vec result;
// static double r1=0;//,r2;        // curve's inner and outer radian ( with space )
// static double nex_r1=0,nex_r2;   // and the same for next curve
static double r3=0; //nex_r3=0;     // driving line's radian
static double nex_yli=0;            // how far the driving line is from end of straight
static int n=0;                     // current segment's number
static int rata_alustettu=false,liian_iso_ratataul=false;
// static int kulmaerror=0;

static int lahto=true,kierr_paivitys;
// static int saa_ohittaa=true;
static int out_count = 0;
static int johtokierrokset=0;
static int after_pit = 0;

static double max_fuel,best_lap;

static double seg_kulma,car_x=0.0,car_y=0.0;

static char trackname[20];
static char qualtrackname[25];

static int MAX_AJOLINJOJA=0;

//static void* viim_kaytetty;

static double *sisa_r;

static double time_count;


struct ajolinjasto {
  double r;          // ajolinjan s„de
  double x,y;        // ajolinjan keskipisteen koordit
  vector tangentti;  // tangentti ajolinjalta seuraavalle
  ajolinjasto *seur;
  ajolinjasto *edel;
};


static ajolinjasto *ajolinjat;
static ajolinjasto *nyk_ajol_os;
static int NAJOL;
// static int VIIM_AJOL;

static vector X;
static vector sijainti;
static vector nopeus;
static vector seg_yks_v;
static vector nyk_tangentti;


/***************************************************************************/
//                                                                         //
//             FUNKTIOIDEN ESITTELYT                                       //
//                                                                         //
/***************************************************************************/

static double sqr(double x);
// static inline int xcon(double x);
// static inline int ycon(double y);
static double vektorien_kulma(vector a,vector b);
static vector r3_keskipiste(int n,CURVE_TYPES tyyppi,double ajor,double r);

static vector laske_xy_ympyralla(vector a,double r3);
static vector pisteesta_ympyralle(vector R3,double r3,vector C);

static double driving_lines_rad(int tyyppi,double r,double omega,double vara,double sisavara);
// static void all_driving_lines(void);

static double laske_s(double d,double r1,double r2);
// static vector ajol_valin_tangentti(vector d_ajol,void *os);

static CURVE_TYPES kurvintyyppi(int n);
static void laske_kaikki_tangentit(void);
// static void lisaa_ajolinja(ajolinjasto *nyky);
// static ajolinjasto* poista_tyhjat_taulukosta(ajolinjasto *nyky,ajolinjasto *ensim_ajol);
static void linkita_ajol_taulukosta(void);
static void track_init(situation& s);


static void laskeperusjutut(situation& s);
static double speed_for(double r3,double d);
static void speed(double v);
static double uusi_alpha(void);
static void vaistele(situation &s);
static void sladiraj(double nykyv);
static void slow_for_refueling(situation& s);
static void my_stuck(situation& s);


/**************************************************************************/
//                                                                        //
//             VECTORS-LUOKAN METODIT                                     //
//                                                                        //
/**************************************************************************/

vector::vector(){  // constructor nollaa tilanteen
  x=0;
  y=0;
  l=0;
}

void vector::set(double new_x,double new_y){  // initill„ vaihdetaa vektorin sis„lt”„
  x=new_x;
  y=new_y;
  l=sqrt(x*x+y*y);
}

void vector::turn(double alfa){
  double vx=x,vy=y;
  double sini=sin(alfa);
  double kosini=cos(alfa);

  x=kosini*vx-sini*vy;
  y=sini*vx+kosini*vy;
}
double vector::get_x(){ return x;} // palauttaa x:n
double vector::get_y(){ return y;} // palauttaa y:n
double vector::get_l(){ return l;} // palauttaa pituuden
/*
static int huijaus=0;
extern Car* pcar[16];// huijauksessa siirret„„n autoa Car::movecar funktiolla
*/

//*************************************************************************/
//                                                                        //
//                       KOODI ALKAA                                      //
//                                                                        //
/**************************************************************************/
/*
  static inline double sign(double x){
  if(x < 0.0) return -1.0;
  return 1.0;
  }*/
/* functio laskee radan suunnan (pisteessa jossa ollaan menossa) 
   suuntaisen yksikkovektorin */
/*Vector track_dir(Situation &s){
  Vector dir;
  segment cur = trackin[s.seg_ID];
  if(s.cur_rad == 0.0){
  dir.set_xy(cur.end_x-cur.beg_x,cur.end_y - cur.beg_y);
  } else {
  // if these assertions fail, code below them is wrong!
  assert(s.to_end >= 0.0);
  assert(cur.length >= 0.0);
  double sign = sign(cur.radius);
  dir.set_dir_len(cur.beg_ang + sign*(cur.length - s.to_end), 1.0);
  }
  }
*/

/**************************** x toiseen-funktio ***************************/
static double sqr(double x){

  return x*x;
}


/**************** laskee vektoreiden v„lisen kulma **********************/
static double vektorien_kulma(vector a,vector b){
double ristitulo,pistetulo,temp;
  ristitulo=a*b;
  pistetulo=(a,b);
  if(a.get_l()==0.0 || b.get_l()==0.0) return 999.0;
  pistetulo/=(a.get_l()*b.get_l()); // cos(alfa)=pistetulo/(a:n pit*b:n pit)
  if(fabs(pistetulo)>1.0)
    return 0.0;
  temp=acos(pistetulo);
  if(ristitulo<0) return -temp;
  else return temp;
}

/*********************    laskee ajolinjan keskipisteen *******************/
static vector r3_keskipiste(int n,CURVE_TYPES tyyppi,double ajor,double r){

// n=segmentin numero jonka ajolinjaa lasketaan
  double kulma=0.0;
  vector keskipiste;
  //lasketaan kulma jolla segmentilt„
  //l„hdet„„n keskipisteeseen
  switch(tyyppi/*curves[n].tyyppi*/){
    case straight:break;
    case norm_curv:
    case norm_shic:
      kulma=lftwall[n].beg_ang+lftwall[n].end_ang;
      // jos mutka on oikealle ja ylitt„nyt nollan:
      if(r<0 && lftwall[n].beg_ang<lftwall[n].end_ang) kulma-=2*PII;
      if(r>0 && lftwall[n].beg_ang>lftwall[n].end_ang) kulma+=2*PII;

      kulma/=2;
      break;
    case first_curv:
    case shic_first_curv:
    case mid_curv:   // mid_curv:lle ihan sama mik„ on kulma
      kulma=lftwall[n].end_ang;
      break;
    case last_curv:
    case last_curv_shic:
      kulma=lftwall[n].beg_ang;
      break;
  }
  kulma+=PII_2;

  double dist=ajor-r;

  if(r<0) dist+=norm_sisavara;
  else if (r>0) dist-=norm_sisavara;

  keskipiste.set( cos(kulma)*dist+lftwall[n].cen_x,
                     sin(kulma)*dist+lftwall[n].cen_y );
  return keskipiste;
}

/********* laskee tangentin x ja y koordinaatit ympyr„n kaarella **********/
//         a_x ja a_y on vektori autosta ympyran keskipisteeseen

static vector laske_xy_ympyralla(vector a,double r3){
double by,bx;
double temp;
double a_2een,r3_2een;
vector b;

  b.set(0.0,0.0);  // nollaa!
  a_2een= sqr(a.get_l());// a_x * a_x + a_y * a_y; // a_x ja a_y on vektori
  r3_2een=r3*r3;                 // ajolinjan keskipisteeseen
  if(r3_2een>=a_2een) return b;
  temp=sqrt(a_2een-r3_2een);
  if(temp<0.00001) temp=0.00001;
  temp=r3/temp;
  by=(a.get_y() - a.get_x() * temp) / (1+ r3_2een/(a_2een-r3_2een));
  bx=a.get_x() + temp * by;
  b.set(bx,by);

  return b;
}

/********************* laskee tangentin absoluuttisen kulman **************/
static vector pisteesta_ympyralle(vector R3,double r3,vector C){
  // ajolinjan keskipiste suhteessa autoon
  vector a=R3-C; // vektori autosta ajolinjan keskipisteeseen
  vector b;      // vektori autosta ajolinja ympyr„lle (tangentin suunt)
  b=laske_xy_ympyralla(a,r3); // pal. xx:„„n ja yy:hyn suuhteelliset
                                    // koordit (autoon n„hden) eli vektorin
                                    // autosta tangentille

//  if(b.get_l()==0.0) return b; // virhe! jos ajolinjan et. on 0

  return b;

}


/**************   laskee ajolinjojen v„lisen tangentin pituuden   **********/
static double laske_s(double d,double r1,double r2){
double s,alfa;

  if(r1*r2 > 0){ // samansuuntaiset ajolinjat
    s=sqr(d)-sqr(r1-r2);
    if (s>0) s=sqrt(s);
    else s=0.0;
  }
  else {        // erisuuntaiset ajolinjat
    r1=fabs(r1);
    r2=fabs(r2);

    s= (r1+r2)/d; // =cos(alfa) = r2/( r2*d/(r1+r2) )
    if(s>=1.0){
      cout << "WappuCar: error 1 (" << s << ")" << endl;
      s=.99999;
    }
    alfa= acos(s);

    s=sin(alfa)*d; // = tangentin pituus
  }
  return s;
}



/****************  laskee vektorin ajolinjojen v„lill„   *******************/
//  d_ajol on ajolinjojen v„linen vektori
//  os on ajolinjan osoite jolta vektori halutaan
static vector ajol_valin_tangentti(vector d_ajol,ajolinjasto *os){

double s,kulma;

  s=laske_s(d_ajol.get_l(),os->r,os->seur->r);

  kulma=s/d_ajol.get_l();

  if (fabs(kulma)>1.0) // jos ei voi laskea palautetaan nollavektori
    { d_ajol.set(0.0,0.0); return d_ajol; }

  if(kulma<0)
    kulma=-kulma;
  kulma=acos(kulma);
  if (s<0.1 && os->r * os->seur->r > 0) { // jos ajolinjat todella l„hell„
                                          // toisiaan ja samansuuntaisia

    s=1.0;
    kulma=PII_2;
//    if (os->r > 0) kulma=-kulma;
  }
//  else // normaalitilanteessa:
    if ( os->seur->r > os->r ) kulma=-kulma; // jos seuraavan ajolinjan s„de
                                             // pienempi, kierto neg. suuntaan

  d_ajol.turn(kulma); // kierret„„n ajolinjojen v„linen vektori niiden
                      // tangentin suuntaiseksi
  d_ajol=(s/d_ajol.get_l())*d_ajol; // muutetaan pituus oikeaksi

  return d_ajol;

}

/*********************    laskee ajolinjan s„teen       *******************/
static double driving_lines_rad(int tyyppi,double r,
                                double omega,double vara,double sisavara){
double r1,r2=0;

  // sisa ja ulkokurvin sateet - vara
  if (r<0){ r1=r-sisavara; r2=r-width+vara;}
  else { r1=r+sisavara; r2=r+width-vara; }

  if (tyyppi==mid_curv)
  return r1;

  if (tyyppi==norm_curv || tyyppi==norm_shic) omega=omega/2;
  return (r2-r1*cos(omega))/(1-cos(omega));
}

/**************** Kurvin tyypin tunnistus *********************************/
static CURVE_TYPES kurvintyyppi(int n){

  CURVE_TYPES tyyppi;

  if (sisa_r[n]==0) tyyppi=straight;

  else if (sisa_r[n]*sisa_r[n-1]>0) //jos ed. kurvi samansuunt.
    if (sisa_r[n]*sisa_r[n+1]>0) tyyppi = mid_curv;
    else if (sisa_r[n]*sisa_r[n+1]<0)

      tyyppi = last_curv_shic;

    else tyyppi = last_curv;

  else if (sisa_r[n]*sisa_r[n-1]==0)  //jos ed. segm. oli suora
    if (sisa_r[n]*sisa_r[n+1]>0) tyyppi = first_curv;//seur. on samansuunt
    else if (sisa_r[n]*sisa_r[n+1]<0) //jos seur. erisuunt.
      tyyppi = norm_shic;
    else tyyppi = norm_curv; //jos seuraavakin on suora

  else  // jos ed. kurvi erisuunt
    if (sisa_r[n]*sisa_r[n+1]>0) //ja seuraava samansuunt
//        if (sisa_r[n+1]*curves[n+2].r>0) //ja seuraavakin
        tyyppi=shic_first_curv;//jos ed. oli norm_curv tai shicane
//        else tyyppi = shic_first_curv;
    else if (sisa_r[n]*sisa_r[n+1]<0)
      tyyppi = norm_shic; //seur ja ed. erisuuntaisia
    else tyyppi = norm_shic; //ed. erisuunt, seur. suora

  return tyyppi;
}


/************   Laskee ajolinjoilta tangenttivektorin seuraavalle **********/
static void laske_kaikki_tangentit(void){

  vector d_ajol;

  // lasketaan jokaiselta ajolinjalta tangenttivektori seuraavalle
  int i=0;
  do {
    d_ajol.set(nyk_ajol_os->seur->x - nyk_ajol_os->x,
               nyk_ajol_os->seur->y - nyk_ajol_os->y);
    nyk_ajol_os->tangentti=ajol_valin_tangentti(d_ajol,nyk_ajol_os);
    nyk_ajol_os=nyk_ajol_os->seur;
    i++;
  } while(i<NAJOL);
}



//***************** lis„„ ajolinjan taulukkoon *****************************
/* Not used
static void lisaa_ajolinja(ajolinjasto *nyky){
  int i,j;

  for(i=0;i<NAJOL;i++) if(&ajolinjat[i] == nyky) break;

  for(j=NAJOL;j>i+1;j--) ajolinjat[j] = ajolinjat[j-1];

  i++; // siirret„„n i uuden ajolinjan kohdalle

  // kopioidaan uusi ajolinja nykyisest„
  ajolinjat[i] = ajolinjat[i-1];

  ajolinjat[i].r += (ajolinjat[i].r<0?-20:20);
  ajolinjat[i].seur = &ajolinjat[i+1];
  ajolinjat[i].edel = &ajolinjat[i-1];

  return;
}
*/

//*************** poistaa taulukosta tyhj„t v„list„ *******************
// palauttaa nykyisen ajolinjan uuden osoitteen
/* Not used
static ajolinjasto* poista_tyhjat_taulukosta(ajolinjasto *nyky,ajolinjasto* ensim_ajol){
  ajolinjasto *temp;
  int i;
  int uusi_indeksi;

  temp = ensim_ajol;

  for(i=0;i<NAJOL;i++){
    // j„rjestet„„n ajolinjat taulukkoon uudestaan (poistetaan tyhj„t)
    if(temp == nyky) uusi_indeksi = i;
    ajolinjat[i] = *temp;
    temp = temp->seur;
  }

  linkita_ajol_taulukosta();

  return &ajolinjat[uusi_indeksi];
}
*/

/***************** linkitt„„ ajolinjat   ***********************************/
//
// eli kertoo jokaiselle ajolinjalle seuraavan ja edellisen osoitteen
static void linkita_ajol_taulukosta(void){
  int i;

  for(i=1;i<NAJOL;i++){
    ajolinjat[i].edel=&ajolinjat[i-1];
    ajolinjat[i-1].seur=&ajolinjat[i];
  }
  ajolinjat[NAJOL-1].seur=&ajolinjat[0];
  ajolinjat[0].edel=&ajolinjat[NAJOL-1];
}

int do_driving_lines(char *track){

  ifstream datafile; // avataan virta tiedostolle!
  char radannimi[25];
  int taulukkoalkioita=0,luettuvaroja=0,indeksi,ajolinjoja=0,i=0,k=0;
  int found = 0;

  datafile.open("vecto071.dat"); // avataan tiedosto
  while (datafile.good()){
    datafile >> radannimi;
    if (strcmp(radannimi,track)==0){
      found = 1;
      datafile >> v_kerr;
      datafile >> accslad;
      datafile >> decslad;
      datafile >> curvslad;
      datafile >> ajolinjakerr;
      datafile >> maxkulma;
      datafile >> vara;
      datafile >> norm_sisavara;
      datafile >> after_pit;
      datafile >> taulukkoalkioita;
      datafile >> ajolinjoja;
      break;
    } else datafile.ignore(120,'\n');
  }

//  if(!found) cerr << "Ei l”ytynynn„ tiedostosta!" << endl;
  sisavara=norm_sisavara;

  a=1.025*g;

  for (i=0;i<NSEG;i++){ // sisakurvin sade:
    if (lftwall[i].radius==0&&rgtwall[i].radius==0) sisa_r[i]=0;
    else if (lftwall[i].radius<0) sisa_r[i]=rgtwall[i].radius;
    else sisa_r[i]=lftwall[i].radius;
  }

  sisa_r[NSEG]=sisa_r[0];
  sisa_r[NSEG+1]=sisa_r[1];
  sisa_r[NSEG+2]=sisa_r[2];
  sisa_r[NSEG+3]=sisa_r[3];

  if(!ajolinjoja){ // jos ajolinjoja ei ole m„„ritelty tiedostossa,
                   // joudutaan itse ne p„hk„ilem„„n

    if(taulukkoalkioita) datafile >> indeksi;

    CURVE_TYPES tyyppi;
    // identify curve types:

    for(i=0;i<NSEG;i++){

    tyyppi=kurvintyyppi(i); // tunnistetaan kurvin tyyppi!

      if(tyyppi!=straight){ // jos ei ole suora, lasketaan ajolinjoja

            vector keskipiste; // ajolinjan keskipiste
            double nyk_vara,nyk_ajor;// t„m„n kurvin vara ja ajolinjan s„de

            // jos tiedostossa on annettu varoja
            if( taulukkoalkioita && indeksi <= i){ // tiedoston ajolinjaindeksi
              datafile >> nyk_vara; // luetaan vara kurville tiedostosta
              luettuvaroja++;
              if(luettuvaroja < taulukkoalkioita) // jos viel„ j„ljell„ varoja,
                datafile >> indeksi; // seuraavan indeksi tiedostosta
              else indeksi=999; // tai jos luettu jo kaikki, indeksi isoksi
            } else nyk_vara=vara; // muutoin yleinen vara

            // voidaan laskea ajolinjan s„de kun tiedet„„n kurvin tyyppi,
            // sis„laidan s„de, kurvin pituus, vara ja sis„vara
            nyk_ajor = driving_lines_rad(tyyppi,sisa_r[i],lftwall[i].length,
                                         nyk_vara,norm_sisavara);

            // ajolinjan keskipiste voidaan laskea, kun tiedet„„n kurvin
            // keskipiste, s„de, pituus ja tyyppi ja ajolinjan s„de
            keskipiste=r3_keskipiste(i,tyyppi,nyk_ajor,sisa_r[i]);

            // siirret„„n lasketut tiedot taulukkoon
            ajolinjat[k].r=nyk_ajor;//curves[i].ajor;
            ajolinjat[k].x=keskipiste.get_x();//curves[i].x;
            ajolinjat[k].y=keskipiste.get_y();//curves[i].y;

/*            if(k>0){ // linkitet„„n edellinen ajolinja nykyiseen ja
                     // nykyinen edelliseen ajolinjaan
              ajolinjat[k].edel=&ajolinjat[k-1];
              ajolinjat[k-1].seur=&ajolinjat[k];
            }*/
            k++;

            if(MAX_AJOLINJOJA < k){ liian_iso_ratataul=true; return 0; }

      }

    }

    NAJOL=k;//+1; // ajolinjojen m„„r„, NAJOL-1 on viimeinen
  }// endif(!ajolinjoja)

  else{ // jos ajolinjat m„„ritelty tiedostossa, tullaan t„nne

    // jos tiedostossa on m„„ritelty sek„ varoja kurveille, ett„ oikeat
    // ajolinjat, siirret„„n tiedosto osoitinta ajolinjojen alkuun
    // lukemalla varat pois alta
    for (i=0;i<taulukkoalkioita;i++){// siirra kurvien 'varat' tiedostosta
      datafile >> indeksi >> indeksi;// hukkaan
    }
    NAJOL=ajolinjoja;         // otetaan ajolinjojen m„„r„ talteen
    if(MAX_AJOLINJOJA < NAJOL){ liian_iso_ratataul=true; return 0; }

    for(i=0;i<ajolinjoja;i++){// sijoitetaan ajolinjat tiedostosta taulukkoon
      datafile >> ajolinjat[i].x;
      datafile >> ajolinjat[i].y;
      datafile >> ajolinjat[i].r;
    }
  }

  // suljetaan tiedosto
  datafile.close();

  // linkitet„„n ajolinjat
  linkita_ajol_taulukosta();
  return !found;
}

/********************** Radan tunnistus ja kurvitaulukon alustus ***********/
static void track_init(situation& s){

// int i=0,k=0;
void *viim_kaytetty;

  X.set(1.0,0.0);
  n=0;
  strcpy(trackname,get_track_description().sName);
  strcpy(qualtrackname,"qual_");
  strcpy(&(qualtrackname[5]),get_track_description().sName);

  NSEG=get_track_description().NSEG;
  lftwall=get_track_description().lftwall;
  rgtwall=get_track_description().rgtwall;
  width=get_track_description().width;

  sisa_r=(double*)(s.data_ptr);
  viim_kaytetty = sisa_r + NSEG + 4; // TOIMIIKO? !

  ajolinjat=(ajolinjasto*)(viim_kaytetty);

  MAX_AJOLINJOJA=( 4096-sizeof(sisa_r[NSEG+4]) )/sizeof(ajolinjasto);

//  if(MAX_AJOLINJOJA<NSEG/2){ liian_iso_ratataul=true; return; }

  if((s.stage == QUALIFYING && do_driving_lines(qualtrackname) )
     || s.stage != QUALIFYING)
     do_driving_lines(trackname);

  // asetetaan nykyiseksi ajolinjaksi ensimm„inen
  nyk_ajol_os=ajolinjat;

  // lasketaan kaikkien ajolinjojen v„liset vektorit
  laske_kaikki_tangentit();

  rata_alustettu=true;
}

/******************    laskeskelee helppoja juttuja    ********************/
static void laskeperusjutut(situation& s){
double d_cen;
  sisavara=norm_sisavara;

  if (s.lap_flag && kierr_paivitys!=s.laps_to_go){
    if (s.position==0) johtokierrokset++;
    if (s.lap_time < best_lap || best_lap==0) best_lap=s.lap_time;
    else johtokierrokset=0;
    kierr_paivitys=s.laps_to_go;
  }

  n=s.seg_ID;  // current segment's number
  if (n==1 || s.to_end<nex_yli) lahto=false;

  if(s.to_rgt < 0.0 || s.to_lft < 0.0) out_count++; else out_count = 0;

  if(s.cur_rad!=0){ // jos kurvissa
    // lasketaan et„isyys kurvin keskipisteeseen
    // ja lasketaan radan keskiviivan absoluuttinen kulma
    if(s.cur_rad<0){
      d_cen=s.cur_rad-s.to_rgt;
      seg_kulma=lftwall[n].end_ang+s.to_end;
    }
    else {
      d_cen=s.cur_rad+s.to_lft;
      seg_kulma=lftwall[n].end_ang-s.to_end;
    }
    if(seg_kulma>2*PII) seg_kulma-=2*PII;
    if(seg_kulma<-2*PII) seg_kulma+=2*PII;

    // lasketaan x ja y koordinaatit autolle
    car_x=cos(seg_kulma-PII_2)*d_cen+lftwall[n].cen_x;
    car_y=sin(seg_kulma-PII_2)*d_cen+lftwall[n].cen_y;

  } else { // jos suoralla
    // lasketaan radan keskiviivan absoluuttinen kulma
    seg_kulma=lftwall[n].end_ang;

    double sini=sin(seg_kulma);
    double cosini=cos(seg_kulma);

    // kuinka pitk„sti suoraa on jo edetty
    double et_alusta=s.cur_len-s.to_end;

    // x ja y koordit autolle
    car_x=lftwall[n].beg_x+cosini*et_alusta+s.to_lft*sini;
    car_y=lftwall[n].beg_y+sini*et_alusta-s.to_lft*cosini;
  }

  seg_yks_v.set(cos(seg_kulma),sin(seg_kulma));// x ja y saadaan kulman avulla
  seg_yks_v=seg_yks_v/seg_yks_v.get_l();      // muutetaan yksikk”vektoriksi


  sijainti.set(car_x,car_y);

  double vy=s.vn;
  double vx=sqrt(s.v*s.v-vy*vy);
  if (s.v<0) vx=-vx;

  nopeus.set(vx,vy);
  nopeus.turn(seg_kulma);
}



/*********** antaa nopeuden jolla voidaan ajaa tiettyyn kurviin ***********/

static double speed_for(double r3,double d){



double v0;

  if (r3==0) return 1000;

  v0=v_kerr*sqrt(fabs(r3));

  return sqrt(v0*v0+2*a*d);

}



/************   laskee nopeuden jota t„ss„ pisteess„ voidaan ajaa *********/

static void speed(double v){

double d,s,kulma;
  vc=1000;

  s = nyk_tangentti.get_l();
  // d on matka nykyisen ajolinjan keskustaan
  d = sqrt(sqr(s)+sqr(nyk_ajol_os->r));
  if (d-fabs(nyk_ajol_os->r) < 20.0 /*|| fabs(alpha)>.04*/ ) // jos l„hell„ ajolinjaa
    vc=min(vc,speed_for(nyk_ajol_os->r,0));
  else
    vc=min( vc , speed_for(nyk_ajol_os->r,s) );

  kulma=vektorien_kulma(nyk_tangentti,nyk_ajol_os->tangentti);
  if(kulma==999.0){

    kulma=0; // virhe kulman laskussa!
  }

  if( nyk_ajol_os->r * kulma<0 ){  // ajolinjan s„de ja
                                  // k„„ntymiskulma erimerkkiset
    double temp,temp2;
    temp=vektorien_kulma(nyk_ajol_os->edel->tangentti ,
                            nyk_ajol_os->tangentti);
    temp2=vektorien_kulma(nyk_ajol_os->edel->tangentti ,
                            nyk_tangentti);
    // onko kurva pitempi kuin PII?
    if(nyk_ajol_os->r*temp<0 || fabs(temp2)+fabs(temp) > PII)
      kulma=2*PII-fabs(kulma);      // => k„„ntyy enemm„n kuin PII
    else {
      nyk_ajol_os=nyk_ajol_os->seur; // muutoin ollaan v„„r„ll„ ajolinjalla!
      kulma=0;
    }
  }
  double matkakerroin=sqr(1.5*g)-sqr(/*sin(alpha)*vc*/ v*v/nyk_ajol_os->r);
  if(matkakerroin<=0.0)

    matkakerroin=.00001;

  else matkakerroin=sqrt(matkakerroin)/(1.5*g);

  if(matkakerroin>1.0)

    matkakerroin=1;



  s+=(fabs(kulma*nyk_ajol_os->r)+nyk_ajol_os->tangentti.get_l())

      *matkakerroin;// jos kurvissa, jarrutukseen tarvii enemm„n matkaa

                    // => kerrotaan matkaa olevan v„hemm„n kuin oikeasti on

  vc=min( vc , speed_for(nyk_ajol_os->seur->r,s) );



}



/**************** laskee uuden alphan  ************************************/

static double uusi_alpha(void){//double cur_rad,double nex_rad){

  // seur_kulma on kurvin suunnan ja ajoympyr„n tangentin v„linen kulma

  double nyk_kulma,seur_kulma;
  vector nyk_kesk; // ympyran keskustan sij.
  vector seur_kesk; // ympyran keskustan sij.
  // vector kurv_kesk; // kurvin keskustan sij.
  vector nyk_ajol;     // vektori autosta nykyiselle ajolinjalle
  vector seur_ajol;    // vektori seuraavalle ajolinjalle
  // vector nyk_sisakurv; // vektori nykyisen kurvin sisalaidalle
  
  while(1) {
    // asetetetaan nykyisen ajolinjan keskustan sijainti vektoriksi
    // ( absoluuttiseen koordinaatistoon )
    // ja lasketaan tangenttivektori sen ajolinjalle
    nyk_kesk.set( nyk_ajol_os -> x , nyk_ajol_os -> y);
    nyk_ajol = pisteesta_ympyralle( nyk_kesk , nyk_ajol_os -> r , sijainti);
    
    // sama seuraavalle ajolinjalle
    seur_kesk.set( nyk_ajol_os -> seur -> x , nyk_ajol_os -> seur -> y);
    seur_ajol = pisteesta_ympyralle( seur_kesk , nyk_ajol_os -> seur ->r ,
				     sijainti);
    
    // jos virhe, segmentin suunnan ja nopeusvektorin suunnan v„l. kulma
    if(nyk_ajol.get_l()==0.0) nyk_kulma=vektorien_kulma(nopeus,seg_yks_v);
    // jos ei virhett„
    else nyk_kulma = vektorien_kulma(nopeus,nyk_ajol);
    
    //lasketaan kulma seuraavalle ajolinjalle:
    seur_kulma = vektorien_kulma(nopeus,seur_ajol);
    
    // muutetaan ajolinjojen keskustojen koordit suhteellisiksi auton paikkaan
    nyk_kesk=nyk_kesk-sijainti;
    seur_kesk=seur_kesk-sijainti;
    
    if(out_count > 5 && (nyk_kesk,seg_yks_v)<0.0){
      nyk_ajol_os = nyk_ajol_os->seur;
      nyk_tangentti = seur_ajol;
      out_count = 0;
	  continue;
    }
      
    // tutkitaan voidaanko edes ajatella ajolinjan vaihtoa
    if(nyk_kesk.get_l()-fabs(nyk_ajol_os->r)< width//ollaan lahella ajolinjaa
       && seur_ajol.get_l()!=0.0){ // ja ei olla seuraavan ajolinjan sisalla
      
      if (vektorien_kulma(nyk_ajol,seur_ajol) *
	  vektorien_kulma(nyk_ajol,nyk_kesk) < 0
	  && fabs(vektorien_kulma(nyk_ajol_os->tangentti,nyk_ajol))<.2 )
	{
	  nyk_ajol_os=nyk_ajol_os->seur;// jos ei leikkaa nykyist„, vaihdetaan
	  // ajolinjaa,
	  nyk_tangentti=seur_ajol;      // paivitetaan nyk. tangentti
	  continue;
	  //return seur_kulma;            // ja palautetaan uusi alpha
	}
      // jos per„kk„iset ajolinjat samansuuntaisia:
      // r*seur_r>0 jos kummatkin negatiivisia tai kummatkin positiivisia:
      if( nyk_ajol_os->r * nyk_ajol_os->seur->r > 0) {
	
	vector d_ajol; // ajolinjojen keskustojen v„linen vektori
	double d_r;     // ajolinjojen s„teitten pituuksien erotus
	
	d_ajol = seur_kesk - nyk_kesk;
	d_r = fabs( nyk_ajol_os->r ) - fabs( nyk_ajol_os->seur->r );
	
	if(   nyk_ajol.get_l()!=0
	      && d_r > 0   //  seuraavan ajolinjan s„de pienempi
	      && fabs(d_r-d_ajol.get_l()) < 0.001 ) {
	  
	  if(( nyk_ajol_os->r > 0 && vektorien_kulma(d_ajol,nyk_ajol) > PII_2 )
	     ||( nyk_ajol_os->r < 0 && vektorien_kulma(nyk_ajol,d_ajol) > PII_2 )
	     ) {
	    nyk_ajol_os=nyk_ajol_os->seur;
	    nyk_tangentti=seur_ajol;
	    continue;
	    //return seur_kulma;
	  }
	}
      }
      
      double kulma=vektorien_kulma(nyk_kesk,seur_ajol);
      
      if( kulma*nyk_ajol_os->r<0 && fabs(kulma)>PII_2 ){
	nyk_ajol_os=nyk_ajol_os->seur;
	nyk_tangentti=seur_ajol;
	return seur_kulma;
      }
    }
    nyk_tangentti=nyk_ajol;
    return nyk_kulma;
  }
}

static int ristitulo(double x1,double y1,double x2,double y2){

  return (int)(x1*y2-y1*x2);

}



static int tormaa(rel_state r){

int neg,pos;



  neg=0;pos=0;



  if (ristitulo(r.rel_x-1.1*CARWID,r.rel_y-1.1*CARLEN,r.rel_xdot,r.rel_ydot)<0) neg++;else pos++;

  if (ristitulo(r.rel_x+1.1*CARWID,r.rel_y-1.1*CARLEN,r.rel_xdot,r.rel_ydot)<0) neg++;else pos++;

  if (ristitulo(r.rel_x-1.1*CARWID,r.rel_y+1.1*CARLEN,r.rel_xdot,r.rel_ydot)<0) neg++;else pos++;

  if (ristitulo(r.rel_x+1.1*CARWID,r.rel_y+1.1*CARLEN,r.rel_xdot,r.rel_ydot)<0) neg++;else pos++;



  // jos kaikki ristitulot samanmerkkisi„, nopeusvektori menee autosta ohi

  if(neg==3 || pos==3) return 0;



  return 1; // ja muutoin osuu

}





static void vaistele(situation &s){



// T„m„n tiedon mukaan toimitaan!
/* struct rel_state {
   double rel_x;     // how far to your right is the car ahead?
   double rel_y;     // how far in front of you?
   double rel_xdot;  // how fast cutting accross from left-to-right?
   double rel_ydot;  // how fast is he getting away from you?
   int who;          // an identifier for that car.
};
*/

int i,k;
int tormaajat[3],tormaajia=0;
rel_state r;
double dist[3],v[3],closing_v,time;

  // t„ss„ luupissa k„yd„„n l„pi kaikki autot jotka n„hd„„n ja katsotaan
  // t”rm„t„„nk” niihin ja laitetaan t”rmaajat taulukkoon
  for(i=0;i<3;i++){

    r=s.nearby[i];
    if (r.who==999) continue;

    double dot = r.rel_x * r.rel_xdot + r.rel_y * r.rel_ydot;//compute dot product of vectors
    if(dot >= 0.0)            // no action if car is not approaching.
       continue;

    dist[i]=sqrt(sqr(r.rel_x)+sqr(r.rel_y));      // auton et„isyys
    v[i]=sqrt(sqr(r.rel_xdot)+sqr(r.rel_ydot));   // auton l„h. nopeus

    tormaajat[tormaajia]=(tormaa(r)?i:0);
    if(tormaajat[tormaajia]) tormaajia++;

    if (dist[i]<5*CARLEN && r.rel_ydot<0 && v[i]-sqrt( dist[i] / 0.16 )>10 && v[i]<s.v) vc=10;


  }
  if(!tormaajia) return;

  // jos tullaan t„nne asti ainakin yksi auto on t”rm„yskurssilla
  for(k=0;k<tormaajia;k++){

    i=tormaajat[k];
    r=s.nearby[i];  // koodin kauneuden vuoksi

    double dot = r.rel_x * r.rel_xdot + r.rel_y * r.rel_ydot;//compute dot product of vectors

    // nopeusvektorin ja paikkavektorin v„linen kosini
    // = a piste b / (  a:n pit * b:n pit )
    // => b:n (nop.vekt) projektio paikkavektorilla eli l„hestymisnopeus
    //    = a piste b / a:n pit = b*cos(alfa)
    if(dist[i]>0.0001 && fabs(dot)>0.0001){

      closing_v=dot/dist[i];

      time=dist[i]/fabs(closing_v);

      if (time>2) continue;
    }
    else {
      closing_v=0.0;
      //      cout << " NOLLA!!! ";
      continue;
    }

    // jos tullaan t„nne t”rm„„miseen kuluva aika time < 3 sekuntia
    vc=min(vc,.85*(s.v-v[i]));

  }

}



/********** est„„ renkaiden liiallisen lukitsemisen ja suttaamisen ********/
static void sladiraj(double nykyv){

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

/**********   tsekkaa josko bensaa on v„h„n ja ajaa laitaan  *************/
static void slow_for_refueling(situation& s){
  // menn„„n laitaan tankkaamista varten
  if (s.fuel<0.003) {
    if (s.v<40) vc=1000;
    else vc=0;
    if (s.v<v_kerr*sqrt(fabs(s.cur_rad)) || s.cur_rad==0){
      /*if (s.v<80 && s.cur_rad==0)*/ alpha=-asin(s.vn/s.v)-2*s.to_rgt/exp(.15*s.v);
      //else if (s.v<80) alpha=-asin(s.vn/s.v) -.2;
    }
    else if (s.to_rgt>.5*width){
      if (s.cur_rad<0.0) alpha=-asin(s.vn/s.v) -2*s.to_rgt/exp(.15*s.v);
      else alpha=-asin(s.vn/s.v)-s.to_rgt/exp(.2*s.v);
    }
    if (s.cur_rad==0 && s.to_rgt>5 && s.to_end>1.2*nex_yli) alpha=-.2-s.vn/s.v;
  }
}

static void my_stuck(situation& s){

  if ( s.to_lft < 0 || s.to_rgt < 0) {
    if (s.v < 40 || s.to_lft<-.5*width || s.to_rgt<-.5*width) {
      vc = s.v+40;
      if (s.to_lft < 0) alpha=-1-asin(s.vn/s.v);
      if (s.to_rgt < 0) alpha= 1-asin(s.vn/s.v);

      if (s.fuel+0.5>max_fuel && !lahto){
        if (alpha>.1) alpha=.1;
        else if (alpha<-.1) alpha=-.1;
      }
    }
    else {
      if (s.cur_rad==0) vc=s.v+15; else vc=s.v+15;
      if (s.to_lft < 0) alpha=-.3-asin(s.vn/s.v);
      if (s.to_rgt < 0) alpha= .3-asin(s.vn/s.v);
      if (fabs(alpha)*s.v>.1*g) alpha=(alpha<0?-.1:.1)*g/s.v;
    }

  }
}


/**************************************************************************/
//                                                                        //
//                          PAAOHJELMA                                    //
//                                                                        //
/**************************************************************************/
con_vec Vector(situation& s)
{
  static double last_fuel = 9999.9; // big initial value
  static Stage prev_stage = (Stage)-1 ;
  int pit_entry;
  result.request_pit = 0;
  time_count=s.time_count;

  pit_entry = 0;
  if(s.fuel > last_fuel) pit_entry = 1;
  last_fuel = s.fuel;

  if(pit_entry)
    nyk_ajol_os = &ajolinjat[after_pit];


  if (init_flag){                 // kerrotaan robotin nimi,
	my_name_is(name);       // vain 1. kierroksella
	result.fuel_amount = 20;
    result.vc=80;
    result.alpha=0;
    init_flag=false;
    return result;
  }
  // check if stage has changed
  if(s.stage != prev_stage){
      if(s.stage != FINISHED)
          rata_alustettu = 0;
      prev_stage = s.stage;
  }
  
  if (s.starting) {
    best_lap=0;
	if(s.stage == QUALIFYING) result.fuel_amount = 20;
	else result.fuel_amount = 150;
    lahto=true;
    johtokierrokset=0;
//    s.side_vision=true;
    // lasketaan radan ominaisuuksia
    if (!rata_alustettu) track_init(s);

    // alustetaan nyk_ajol_os
    nyk_ajol_os=ajolinjat;

  } else { if(s.stage == QUALIFYING) rata_alustettu = 0;}
  if (liian_iso_ratataul || (s.laps_to_go==0 && s.stage==RACING)){
    result.vc=50;
    double temp=exp(.1*(s.to_lft-.5*width))-1;
    if (temp<-1) temp=-1; else if (temp>1) temp=1;
    result.alpha=temp-s.vn/s.v;
    return result;
  }

  // lasketaan esim. r1,r2, nex_samat, kulmlopp, nykyv
  laskeperusjutut(s);

  // uusi alpha
  alpha=uusi_alpha();

  // lasketaan uusi "velocity commanded"
  speed(s.v);

  // v„istet„„n t”rm„yskurssilla olevia autoja
//  passing(s);

  slow_for_refueling(s);

  vaistele(s);
  
  // prevent tire locking and sliding
  sladiraj(s.v);

  // estet„„n ulosmenoa ja tullaan takaisin tielle jos ollaan ulkona
  // this is my own stuck() function
  my_stuck(s);

  if (s.damage > 20000 || s.fuel < 10){
    result.request_pit = 1;
    result.repair_amount=s.damage;
    result.fuel_amount = 150;
  }

  result.vc = vc;
  result.alpha = alpha;
  return result;
}
