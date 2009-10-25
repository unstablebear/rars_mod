#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "car.h"

//#define debug

const char name[] = "Magic";      // This is the robot driver's name!
static int init_flag = 1;          // cleared by first call
con_vec resultm;                    // This is what is returned.
double largeur;
double vc,alpha;
double dis_fre;
double ent_fre;
double gain;
double nex_vit;
static double pos_vir;
double pos_ent;
double pos_sor;
double k1,k2;
double ap;

double v1;
double v2;
double v3;
double v4;

double l1;
double l2;
double l3;
double l4;


double cr,cl,nr,nl,ar,al,aar,aal;
double te,dist;

int max_seg,seg;
double last_rad,last_len,last_pos,last_vc;
int last_seg;
int fill_tab=0;

static int type_circuit=1;

int mem_virage=0;
double fr=0;
//-----------------------------------------------------------------------------

double abs( double v){
  if( v<0 )
    return -v;
  return v;
}
//-----------------------------------------------------------------------------
double min( double a, double b)
{
  if ( a<b)
    return a;
  else
    return b;
}
//-----------------------------------------------------------------------------
double max( double a, double b)
{
  if ( a<b)
    return b;
  else
    return a;
}
//-----------------------------------------------------------------------------
double lineaire( double val,double sa, double ea,double sb,double eb)
{
  if( sa < ea )
    {
      if( val < sa )
	return sb;
      if( val > ea )
	return eb;
    }
  else
    {
      if( val < ea )
	return sb;
      if( val > sa )
	return eb;
    }
  return (val-sa)/(ea-sa)*(eb-sb)+sb;
}
//-----------------------------------------------------------------------------
int mem_vir( double r1,double r2)
{
  if( r1<0 )
    if( r2 < 0)
      return 1;
    else
      return 0;
  if( r1 > 0)
    if( r2 > 0)
      return 1;
    else
      return 0;
  return 0;
}
//-----------------------------------------------------------------------------
int I(int i)
{
  if( max_seg != 0 )
    return i%max_seg;
  else
    return 0;
}
//-----------------------------------------------------------------------------
double cal_log( double r, double l)
{
  if( r!=0 )
    return abs(r*l);
  else
    return abs(l);
}
//-----------------------------------------------------------------------------
double rayon( double rad,double len )
{
  if ( rad == 0 )
    return 0;
  double coef;
  double l=cal_log( abs(rad), abs(len));
  if( l < 2*largeur )
    {
      coef=lineaire( l, 0 , 2*largeur, 1 , .4);
      return lineaire( abs(len) , 0 , PI , PI / abs(len) * 
(abs(rad)+largeur/2),abs(rad)+largeur)*coef;
    }
  if( abs(len) < PI / 3 )
    {
      return lineaire( abs(len) , 0 , PI/3 , PI / abs(len) * (abs(rad)+largeur/2), 
abs(rad)+largeur/2)*.7;
    }
  return (abs(rad)+largeur/1);
}
//-----------------------------------------------------------------------------
double cal_vit_vir( double r)
{
  if( r!=0 )
    return max(50,sqrt(abs(r)*g));
  else
    return 1000;
}
//-----------------------------------------------------------------------------
double cal_vit_max( double rad,double len,double rad2,double len2)
{
  if( rad != 0)
    {
      if(!mem_vir(rad,rad2))
	return cal_vit_vir( rayon(rad,len) );
      else
	return cal_vit_vir( abs(rad)+largeur/1.5);
    }
  else
    return cal_vit_vir( abs(rad));
}
//-----------------------------------------------------------------------------
double cal_gai(double crad,situation s )
{
  if( crad !=0 )
    return lineaire(abs(crad),20,1000,2,1)*
      lineaire(s.v,0,150,2,.5)*
      lineaire(abs(pos_vir-s.to_lft/largeur),0,1,.15,1.5);
  else
    return 1;
}
//-----------------------------------------------------------------------------
double cal_dis_fre(situation s)
{
  double coef;
  double d5=dist/2;

  if( cr!= 0 )
    coef= lineaire( abs(cl) , 0, PI , .95 , 1.3)*lineaire( s.v, 0 , 150 , 1 , 2 );
  else
    coef=.95;
  
  if( v2 < s.v )
    {
      ent_fre=1;
      return ( s.v*s.v - v2*v2 )/(2*g)*coef+d5;
    }
  else
    if( v3 < s.v )
      {
	ent_fre=2;
	return ( s.v*s.v - v3*v3 )/(2*g)*coef+d5-l2;
      }
  else
  if( v4 < s.v )
    {
      ent_fre=3;
      return ( s.v*s.v - v4*v4 )/(2*g)*coef+d5-l2-l3;
    }
  else
    {
      ent_fre=0;
	return -1;
    }
}
//-----------------------------------------------------------------------------
double ral_vir(situation s )
{
  double deb=1;
  double mid=1;
  double maxi;
  if ( v2<v1 )
    maxi=1;
  else
    maxi=1.3;
  double coef=lineaire( cal_log(cr,cl),0,2*largeur,3,1);
  double end=lineaire( abs(cl) , 0 , PI , maxi , 1 )*coef;
  if( cr !=0 )
    {
      if ( te > .5 * l1 )
	return lineaire( te , l1 , .5*l1 , deb , mid );
      else
	return lineaire( te , .5 * l1 , 0 , mid , end );
    }
  else
    return 1;
}
//-----------------------------------------------------------------------------
double cal_vit( situation s )
{
  if( te < dis_fre )
    {
      if( ent_fre==1 )
	return ral_vir(s)*v2;
      if( ent_fre==2 )
	return ral_vir(s)*v3;
      if( ent_fre==3 )
	return ral_vir(s)*v4;
    }
  return ral_vir(s)*v1;
}
//-----------------------------------------------------------------------------
double int_vir( double rad,double deg )
{
  if( rad > 0 )
    return 0.05+deg/200;
  else
    return 0.95-deg/200;
}
//-----------------------------------------------------------------------------
double ext_vir( double rad,double deg )
{
  if( rad > 0 )
    return .95-deg/200;
  else
    return .05+deg/200;
}
//-----------------------------------------------------------------------------
double cal_ent(double rad,double len)
{
  return lineaire( abs(len), 0 ,PI , int_vir( rad,20) , int_vir(rad,50));
}
//-----------------------------------------------------------------------------
double cal_ent_vir(situation s)
{
  return .5;
}
//-----------------------------------------------------------------------------
double cal_sor(situation s)
{
  int coef=(int)lineaire(s.v,50,90,0,100);
  if( nr==0 )
    return lineaire( abs(cl) , 0 , PI/2 , int_vir(cr,0),ext_vir(cr,coef));
  return lineaire( abs(nl), 0 , PI , .5 , int_vir(nr,25));
}
//-----------------------------------------------------------------------------
double cal_int(situation s)
{
  return .5;
}
//-----------------------------------------------------------------------------
double cal_pos_dro_vir(situation s)
{
  dist = largeur * lineaire ( abs(nl) , PI/4  , PI , 1.8 , 1 );
  double static posi;
  double static test=1;
  double ext=lineaire( nl, 0 ,PI , ext_vir( nr,100) , ext_vir(nr,15));
  if ( te > dist + largeur )
    {
      test=0;
      posi=lineaire( te , l1 , dist  , last_pos , ext);
      return lineaire( te , l1 , dist  , last_pos , ext);
    }
  else
    {
      if( te > dist )
	{
	  test=0;
	  posi=ext;
	  return posi;
	}
      else
	{
	  if( test )
	    posi=last_pos;
	  return lineaire( te , min( l1 , dist ) , 0 , posi , cal_ent(nr,nl));
	  test=1;
	}
    }
}
//-----------------------------------------------------------------------------
double cal_pos_vir( situation s )
{
  if( nr != 0)
    dist = largeur * lineaire ( abs(nl) , PI/4  , PI , 1.8 , 1 );
  else
    dist=0;
  double static posi;
  double static test=1;
  if( cr!=0 )
    {
      if( te > .5 * l1 )
	{
	  test=0;
	  posi=lineaire( te , l1 + largeur *2  , .5*l1 , last_pos , int_vir(cr,0));
	  return lineaire( te , l1 + largeur *2  , .5*l1 , last_pos , int_vir(cr,0));
	}
      else
	{
	  if( te < dist )
	    {
	      test=1;
	      return lineaire( te , min( .5*l1 , dist ) , 0 , posi , cal_ent(nr,nl));
	    }
	  else
	    {
	      test=0;
	      posi=lineaire( te , .5*l1 , 0 , int_vir(cr,0), cal_sor(s));
	      return lineaire( te , .5*l1 , 0 , int_vir(cr,0), cal_sor(s));
	    }
	}
    }
  else
    return cal_pos_dro_vir(s);
}
 //-----------------------------------------------------------------------------
double vit_oval( situation s )
{
  double r;
  if( cr ==0 )
    r=0;
  else
    if(abs(cl) < PI/4 )
      r= PI / abs(cl) * abs(cr);
    else
      r=abs(cr)+largeur/1;
  v1=cal_vit_vir(r);
  
  if( nr ==0 )
    r=0;
  else
    if(abs(nl) < PI/4 )
      r= PI / abs(nl) * abs(nr);
    else
      r=abs(nr)+largeur/1;
  v2=cal_vit_vir(r);
  
  if( v2 < s.v )
    dis_fre=( s.v*s.v - v2*v2 )/(2*g);

  double mid;
  mid=lineaire( v1, 130 , 150 , .95 , 1);
  double v;
  if( cr !=0 )
    {
      if ( te > .5 * l1 )
	v=lineaire( te , l1 , .5*l1 , 1 , mid );
      else
	if( te < .25*l1 )
	  v=lineaire( te , .25*l1 , 0 , mid , 10 );
      else
	v=mid;
    }
  else
    v=1;

  if( te < dis_fre )
    return v2;
  return v1*v;
}
//-----------------------------------------------------------------------------
double pos_oval_dro(situation s)
{
  dist = largeur * lineaire ( abs(nl) , PI/4  , PI , 4 , 2 )*
    lineaire( s.v , 50 , 200 , 1, .5);
  double static posi;
  double static test=1;
  double ext=lineaire( nl, 0 ,PI , ext_vir( nr,100) , ext_vir(nr,15));
  if( l1< 4*dist )
    ext=last_pos;
  if ( te > l1-largeur  )
    {
      test=0;
      posi=lineaire( te , l1 , l1-largeur , last_pos , ext);
      return posi;
    }
  else
    {
      if( te > dist )
	{
	  test=0;
	  posi=ext;
	  return posi;
	}
      else
	{
	  if( test )
	    posi=last_pos;
	  return lineaire( te , min( l1 , dist ) , 0 , posi , int_vir(nr,25));
	  test=1;
	}
    }
}
//-----------------------------------------------------------------------------
double pos_oval( situation s )
{
  double ext=lineaire( s.v , 50 , 100 , 10 , 80 );
  double deb,fin;
  deb=.75;
  fin=.25;
  if( mem_vir(cr,nr) )
    ext=180;
  if(( cl < PI / 1.9 )&&(s.v<150))
    {
      deb=.5;
      fin=.5;
      if( nl < 2 * largeur)
	ext=150;
    }
  if( cr!=0 )
    {
      if( te > deb * l1 )
	{
	  return lineaire( te , l1 , deb *l1 , last_pos , int_vir(cr,0));
	}
      else
	{
	  if( te < fin *l1 )
	    {
	      return lineaire( te , fin *l1 , 0 , int_vir(cr,0), ext_vir(cr,ext));
	    }
	  else
	    {
	      return int_vir(cr,0);
	    }
	}
    }
  else
    return pos_oval_dro(s);
}
//-----------------------------------------------------------------------------
double cor_vit( situation s )
{
//   if( abs(pos_vir-s.to_lft/largeur)<.1)
//     return vc+lineaire( abs(pos_vir-s.to_lft/largeur), 0 , .1 , 20 , 0);
  return vc;
}
//-----------------------------------------------------------------------------
con_vec Magic(situation &s)
{
  static int ls;
   if(init_flag)  {            // first time through, only copy name:
      my_name_is(name);        // copy the name string into the host program
      init_flag = 0;
      resultm.alpha = resultm.vc = 0;
      fill_tab=1;
      last_seg=s.seg_ID;
      ls=1;
      largeur=( s.to_lft +s.to_rgt );
      last_pos=s.to_lft/largeur;
      fill_tab=1;
      max_seg=0;
      return resultm;
   }
   
   max_seg=(int)max((double)s.seg_ID,max_seg);
          
   if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &resultm.alpha,&resultm.vc))
     return resultm;
   
   largeur=s.to_lft + s.to_rgt;

   cr=s.cur_rad;
   cl=s.cur_len;

   nr=s.nex_rad;
   nl=s.nex_len;

   ar=s.after_rad;
   al=s.after_len;

   aar=s.aftaft_rad;
   aal=s.aftaft_len;

   te=cal_log( cr , s.to_end);

   l1=cal_log( cr , cl );
   l2=cal_log( nr , nl );
   l3=cal_log( ar , al );
   l4=cal_log( aar , aal );

   if( type_circuit )
     {// Mode Routier
       if(( s.seg_ID==0 )&&(max_seg!=0))
	 {
	   if( ls==max_seg+1)
	     {
	       ls=s.seg_ID+1;
	       last_pos=s.to_lft/largeur;
	     }
	 }
       else
	 if( ls==s.seg_ID )
	   {
	     ls=s.seg_ID+1;
	     last_pos=s.to_lft/largeur;
	   }
       
       v1=cal_vit_max( cr,cl,nr,nl);
       v2=cal_vit_max( nr,nl,ar,al);
       v3=cal_vit_max( ar,al,aar,aal);
   
       if( aar != 0)
	 v4=cal_vit_vir( abs(aar)+largeur/2);
       else
	 v4=cal_vit_vir( abs(aar));
       
       
       if(( fr==0 )&&(cr!=0))
	 fr=cr;

       if((cr!=0)&&(!mem_vir(fr,cr))&&(mem_virage==0))
	 mem_virage=1;

       if((s.lap_flag)&&(mem_virage==0)&&(max_seg!=0))
	  {
	    type_circuit=0;
	  }
       
       dis_fre=cal_dis_fre(s);
       
       gain=cal_gai(cr,s);
       
       pos_vir=cal_pos_vir(s);
       
       vc=cal_vit(s);
       vc=cor_vit(s);
       //   vc=50;
     }
   else
     {//Mode Oval
       if(( s.seg_ID==0 )&&(max_seg!=0))
	 {
	   if( ls==max_seg+1)
	     {
	       ls=s.seg_ID+1;
	       last_pos=pos_vir;
	     }
	 }
       else
	 if( ls==s.seg_ID )
	   {
	     ls=s.seg_ID+1;
	     last_pos=pos_vir;
	   }
       gain=cal_gai(cr,s);
       vc=vit_oval(s);
       pos_vir=pos_oval(s);
     }
   
//    if((s.dead_ahead)&&(cr!=0))
//      {
//        if( nr<0 )
// 	 pos_vir=max(.05,pos_vir-.15);
//        else
// 	 pos_vir=min(.95,pos_vir+.15);
//      }
   
   alpha= gain * ( s.to_lft - pos_vir * largeur ) / largeur;
   alpha-= gain *s.vn / s.v;
   
//   usleep(100);
   resultm.vc = vc;   resultm.alpha = alpha;

if(s.starting) resultm.fuel_amount = MAX_FUEL;
   resultm.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
  resultm.request_pit = 1;
  resultm.repair_amount=s.damage;
  resultm.fuel_amount = MAX_FUEL;
  }

   return resultm;
}









