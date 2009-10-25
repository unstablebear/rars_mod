/* jammer.cpp
** This code is public and jammer.cpp is a novice.
** Kim Laurio, Sweden, 1997.
Return-Path: <kim@ida.his.se>
*/
#include <math.h>
#include "car.h"

static double ACC_S=30.0;
static double ACC_C=24.0;
static double BIASK=0.06;
static double SPDL=50.0;
static double C_INC=0.41;
static double C_INC2=0.35;
static double FAST_C=0.50;
static double L;
static double S=0.5;
static double R;

/* Returns the maximum speed allowed for a segment */
static double getSpd( double rad, double len )
{
  double res;

  if( rad==0.0 ) res = 250.0;
  else res = (rad<0.0) ? sqrt(32.2*(-rad)) : sqrt(32.2*(rad));
  if(len<C_INC) res *= C_INC/len;
  if(len<C_INC2) res *= (C_INC2/len)*(C_INC2/len);
  return res;
}

/* Returns the length of a segment in feet */
static double getLen( double rad, double arc )
{
  if( rad==0.0 ) return arc;
  else return (rad<0.0) ? (-rad)*arc : rad*arc;
}

/* Returns the distance it takes to change speed from v1 to v2 */
static double getDst( double v1, double v2, double acc )
{
  if( v2>v1 ) return 0.0;
  else return 0.5*(pow(v1,2.0)-pow(v2,2.0))/acc;
}

/* Preprocess the given data */
static void processData( double spd[], double len[], double t, situation &s )
{
  spd[0] = getSpd(s.cur_rad,s.cur_len);
  len[0] = t*getLen(s.cur_rad,s.cur_len);
  spd[1] = getSpd(s.nex_rad,s.nex_len);
  len[1] = len[0] + getLen(s.nex_rad,s.nex_len);
  spd[2] = getSpd(s.after_rad,s.after_len);
  len[2] = len[1] + getLen(s.after_rad,s.after_len);
  spd[3] = getSpd(s.aftaft_rad,s.aftaft_len);
  len[3] = len[2] + getLen(s.aftaft_rad,s.aftaft_len);
}

/* Control logic for speed */
static double decideSpd( double spd[], double len[], situation &s )
{
  double t=1.0-(s.to_end/s.cur_len);

  if(s.cur_rad!=0.0) /* We are in a curve... */
  {
    if( s.nex_rad==0.0 && len[1]>getDst(s.v,spd[2],ACC_S) )
    {
      if( len[0]<FAST_C )
      {
        return s.v + 50.0;
      }
      else if( t>0.5 && s.cur_len<3.14 )
      {
        return s.v + t*t*t*t*(spd[1] - s.v);
      }
    }
    if( spd[1]>=spd[0] ) /* and the next segment is faster. */
    {
      return 0.5*(s.v + spd[0]); /* Smooth the transitions */
    }
    else
    {
      if( getDst(s.v,spd[2],ACC_C)>=(len[0]+len[1]) )
      {
        if(s.v>1.05*spd[2])
          return 0.95*s.v;
        else
          return 0.5*(s.v+spd[2]);
      }
      else if( getDst(s.v,spd[1],ACC_C)>=len[0] )
      {
        if(s.v>1.05*spd[1])
          return 0.95*s.v;
        else
          return 0.5*(s.v+spd[1]);
      }
      else
        return spd[0];
    }
  }
  else /* We are on a straight */
  {
    /* Don't slow down for next curve */
    if( s.nex_rad!=0.0 && s.after_rad==0.0 &&
        len[2]>getDst(s.v,spd[3],ACC_S) &&
        s.nex_len<FAST_C )
    {
      return s.v + 50.0;
    }
    /* Look ahead two segments */
    else if( getDst(s.v,spd[2],ACC_C)>=(len[0]+len[1]) )
    {
      if(s.v>1.05*spd[2])
        return 0.95*s.v;
      else
        return 0.5*(s.v+spd[2]);
    }
    /* Look ahead just one segment */
    else if( getDst(s.v,spd[1],ACC_S)>=len[0] )
    {
      if(s.v>1.10*spd[1])
        return 0.90*s.v;
      else
        return 0.5*(s.v+spd[1]);
    }
    else
      return s.v + 50.0;
  }
}

static double decideBias( situation &s, double t )
{
  if(s.cur_rad==0.0)
  {
    if(t<0.2 && s.nex_rad!=0.0)
      return BIASK*(s.v/s.nex_rad);
    else
      return 0.0;
  }
  else
  {
    if(t<0.2 && s.nex_rad!=0.0)
      return BIASK*(s.v/s.nex_rad);
    else
      return BIASK*(s.v/s.cur_rad);
  }
}

static double decidePos( situation &s, double t )
{
  double c, n, p;

  c = (s.cur_rad==0.0)? S: (s.cur_rad<0.0)? R: L;
  n = (s.nex_rad==0.0)? S: (s.nex_rad<0.0)? R: L;
  if(c==0.5)
  {
    p = 1.0-t;
    p *= p;
  }
  else
  {
    p = 1.0-t;
  }
  return c + p*(n-c);
}

con_vec Jammer( situation& s )
{
  static int flag, firstcall = 1;
  static double st, v;
  double p, d, t, w=s.to_lft+s.to_rgt;
  double spd[4], len[4];
  con_vec result = CON_VEC_EMPTY;

  if(firstcall){
    firstcall=0;
    my_name_is( "Jammer" );
    return result;
  }

  if(s.starting){
    flag = 0;
    result.alpha = 0;
    result.vc = 100;
    result.fuel_amount = MAX_FUEL;
    st = s.to_lft/w; /* Save the starting position */
    L = CARWID/w;
    R = (w-CARWID)/w;
    return result;
  }

  if( stuck(s.backward, s.v, s.vn, s.to_lft, s.to_rgt,
            &result.alpha, &result.vc) ) return result;

  t = s.to_end/s.cur_len;
  d = s.to_lft/w;
  processData( spd, len, t, s );

  p = decidePos(s,t);
  if(s.dead_ahead) /* Simple collision avoidance */
  {
    if(d<=0.5)
      if(s.nex_rad<0.0)
        p = R;
      else
        p = L;
    else
      if(s.nex_rad>0.0)
        p = L;
      else
        p = R;
  }
  if( !flag ) /* Keep starting position during acceleration */
  {
    if( t<0.2 ) flag=1;
    else if( t<0.6 ) p = st + (1.0-t)*(1.0-t)*(p-st);
    else p = st;
  }

  result.alpha = exp(-s.v/SPDL)*decideBias(s,t) + (d-p) - (s.vn/s.v);
  v = decideSpd( spd, len, s );
  result.vc = (s.v>45.0) ? v : s.v + 10.0;
result.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
  result.request_pit = 1;
  result.repair_amount=s.damage;
  result.fuel_amount = MAX_FUEL;
  }
	return result;
}


