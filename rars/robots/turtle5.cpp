#include <string.h>
#include <math.h>
#include "car.h"
#include "track.h"
static double CURVESPEEDMULTIPLIER  =    6.0;

static double distancetobraketo(double vnow, double vthen)
{
  if (vnow<vthen)
  {
    return 0.0;  // no need to brake as we are already at the desired speed!
  }
  return .0549 * (vnow+vthen)*(vnow-vthen)/(1.7*2.0);
}


static double curvespeed(double radius)
{
  if (radius == 0.0)
  {
    return(400.0);
  }
  return CURVESPEEDMULTIPLIER * sqrt(fabs(radius));
}


static double curvelength(double len, double rad)
{
  if (rad == 0.0)
  {
	  return (len);
  } 
  else 
  {
	  return (len * fabs( rad )); 
  }
}

con_vec Turtle5(situation &s)
{
  con_vec result = { 0.0f, 0.0f, 0.0f, 0, 0 }; // This is what is returned.
  static int firstcall = 1;

  if (firstcall)
  {                               //  this is the very first call
    my_name_is("Turtle5");        //  this lets everyone know who we are.
    firstcall = 0;                //  theres only one first call
    return result;                //  must return an answer
  }

  if(s.starting) 
  {
    result.fuel_amount = MAX_FUEL;
  }

  double width = s.to_lft + s.to_rgt;
  double track_middle = (s.to_lft+s.to_rgt)/2;
  double directionoftravel = asin(s.vn/s.v);
  double to_end = curvelength(s.to_end, s.cur_rad);

  if (to_end < width)
  {
	  to_end = width;
  }
  
  double ideal_alpha =  atan((s.to_lft - track_middle)  / to_end);

  if (s.cur_rad != 0)  // in other words, we are in a curve
  {
    ideal_alpha =  atan((s.to_lft - track_middle)  / to_end);
  }
  result.alpha = ideal_alpha - directionoftravel;


  result.vc = curvespeed(s.cur_rad);
  if (s.to_end < distancetobraketo(s.v, curvespeed(s.nex_rad)))
  {
    result.vc = 0;
  }

  result.request_pit=0;
  if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
    result.request_pit=1;
    result.repair_amount=s.damage;
    result.fuel_amount=MAX_FUEL;
  }
  return result;                    // here too.
}

