// jas19.cpp - RARS robot driver by Juergen Sang
// jusa@darkstar.bb.bawue.de - Use the source at your own risc
// Should run fine on all race tracks with s1 surface (I hope so :-)
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "misc.h"

// Constants for fine tuning
static const double CORN_SPD_CON = 6.0;     // determines how fast to take corners
static const double SLIP_LIM = 36.0;        // maximum wheel slip wheel_slip()
static const double BIG_SLIP = 5.0;         // Bias factor for steering
static const double MAX_BRAKE_ACCEL = 32.0; // Acceleration on the straight
static const double CURVE_BRAKE_ACCEL = 14.0; // The same in curves
static const double PRE_STEER = 60.0;       // How early steer in corners
static const double PRE_BRAKE = -10.0;      // Bias for brakeing
static const double CURVE_BRAKE=0.5;        // When starting to accelerate
static const double CURVE_ANGLE=PI/5;       //             ""
static const double S_CURVE_MULT=1.2;       // Speed multiplier for S-curve
static const double LONG_CURVE=1000.0;      // Minimum length of a _long_ curve
static const double CURVE_STEER=0.8;        // Steering gain in curve

// Global statics
static int ovt_count = 500;               // Counter for overtaking manuevers
//                                           Set at start to avoid pileup
static double width;                      // track width, feet
static double lane=-1;                    // Target position on the track
static int s_curve=0;                     // Flag for detection of S-Curves
static int s_segid=-100;                  // Segment ID of S-curve
static double s_speed=0;                  // Speed for S-curve
static double slow_down;                  // Slowdown if damaged
static int    last_damage=0;              // Keep track of damage
static int    pile_count=0;               // Counter for pileup avoidance
static int    pile_action=0;              // What to do during avoidance

// Determine how fast to take a corner
static double corner_spd(double radius, double angle, double width)
{
  double result;
  if(radius < 0.0)         // change sign of negative radius
    radius = -radius;
  else if(radius == 0.0)   // This is just insurance, this funtion doesn't
    return(500.0);        // make sense when the radius is zero.
  result =  CORN_SPD_CON * sqrt(radius + .2 * width);
  
  // Speedup for light curves
  if (angle < PI/6)
    result *= 1.2;
  
  // Speedup for wide curves
  double length = (fabs(radius) + 0.5 * width) * angle;
  if (length < 3 * width)
    result *= 1.1;
  
  return result * slow_down;
}

// Softly brake or accelerate
static double wheel_speed(double goal, double present)
{
  double ws;
  
  if(present > goal + 1.0 *SLIP_LIM)  // if too fast,
    ws = present - SLIP_LIM;      // slow down.
  else if(present < goal - 1.0 * SLIP_LIM)  // if too slow,
    ws = present + SLIP_LIM * 2;             // accelerate.
  else                           // if quite close,
    ws = (goal + present) / 2;      // approach desired speed gently.
  
  return ws;
}

// Steering servo
static double steer(situation &s, double gain, double damp)
{
  static double old_lane = -1;
  double l;
  
  // printf("%lf\n",lane);
  // Avoid sharp changes
  if (old_lane >= 0)
    l = (lane + 9 * old_lane)/10;
  else
    l = lane;
  
  old_lane = l;
  
  
  double alpha;
  
  alpha = gain * (s.to_lft - l) / width;
  alpha -= damp * s.vn / s.v;
  
  if (s.cur_rad > 0)
    {
      alpha += BIG_SLIP / s.v;
    }
  else if (s.cur_rad < 0)
    {
      alpha -= BIG_SLIP / s.v;
    }
  
  // printf("%lf %lf %lf %lf\n", alpha, old_lane, lane, l);
  
  return alpha;
}

// Determine distance for braking
static double crit_dist(double v0, double v1, double amax)
{
  double dv;
  
  dv = (v1 - v0);
  
  if (dv > 0.0)
    return 0.0;
  
  return (v0 + 0.5 * dv) * dv / amax;
}



// Main program
con_vec Jas(situation &s)
{
  const char name[] = "Jas";      // This is the robot driver's name!
  static int init_flag = 1;          // cleared by first call
  con_vec result;                    // This is what is returned.
  
  // This paragraph has nothing to do with car control; it is just
  // to identify the driver by copying its name to a global RAM area:
  // This happens only on the very first call to this function
  
  if(init_flag)  {            // first time through, only copy name:
    my_name_is(name);        // copy the name string into the host program
    init_flag = 0;
    result.alpha = result.vc = 0;
    return result;
  }
  
  // Calc special situation (stucked)
  if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
    return result;
  
  //
  // Custom code starts
  //
  
  double speed;                      // target speed for cornering, ft/sec
  double speed_next;                 // target speed for next corner
  double speed_after;
  double speed_aftaft;
  double alpha, vc;                  // components of result
  double to_end;                     // distance to end of segment
  double crit;
  double curve_pos;
  int    i;
  
  // Slow down when damaged
  if (s.damage > 27000)
    slow_down = 0.8;
  else if (s.damage > 24000)
    slow_down = 0.9;
  else
    slow_down = 1.0;

  if (long(s.damage) != last_damage)
  {
    //printf("Damage: %d Diff %d\n", s.damage, s.damage - last_damage);
    last_damage = s.damage;
  }
  
  // Counter for overtaking manuevers
  if (ovt_count > 0)
    ovt_count--;
  
  // find width of track
  width = s.to_lft + s.to_rgt;
  
  // Set initial lane
  if (lane < 0)
    lane = s.to_lft;
  
  // Target speed for current corner
  speed = corner_spd(s.cur_rad, s.cur_len, width);
  
  // Target speed for next corner
  double cur_length;
  double next_length;
  double after_length;
  if (s.cur_rad == 0)
    cur_length = s.cur_len;
  else
    cur_length = (fabs(s.cur_rad) + 0.5 * width) * s.cur_len;
  //printf("%lf\n",cur_length);
  if (s.nex_rad == 0)
    next_length = s.nex_len;
  else
    next_length = (fabs(s.nex_rad) + 0.5 * width) * s.nex_len;
  if (s.after_rad == 0)
    after_length = s.after_len;
  else
    after_length = (fabs(s.after_rad) + 0.5 * width) * s.after_len;
  
  speed_next   = corner_spd(s.nex_rad, s.nex_len, width);
  speed_after  = corner_spd(s.after_rad, s.after_len, width);
  speed_aftaft = corner_spd(s.aftaft_rad, s.aftaft_len, width);
  
  // Special handling for S-curves
  if (s_curve &&
      s.seg_ID != s_segid &&
      s.seg_ID != s_segid+1 &&
      s.seg_ID != s_segid+2)
    {
      //printf("End of S-curve\n");
      s_curve = 0;
      s_segid = -100;
    }
  
  //printf("%lf %lf\n",s.nex_len,s.after_len);
  if (
      // No detection in curves
      (s.cur_rad == 0 || cur_length > LONG_CURVE) &&
      
      // True S-Curve
      ((
	// Both not zero or different sign
	s.nex_rad * s.after_rad < 0 &&
	
	// Next segment is curve, not too tight and there is a straight way
	s.nex_len < PI/2 &&
	(s.nex_rad + width * 0.6) / cos(s.nex_len/2) < s.nex_rad + width &&
	
	// The same for second curve
	s.after_len < PI/2 &&
	(s.after_rad + width * 0.6) / cos(s.after_len/2) < s.after_rad + width
	) ||
       
       // Curve (very light) followed by straight
       (s.nex_rad != 0 && s.after_rad == 0 &&
	s.nex_len < PI/5)
       ))
    {
      //if (!s_curve)
      //printf("S detected !\n");
      s_curve = 1;
      s_segid = s.seg_ID;
      
      // Set speed for this S-curve
      s_speed = speed_next;
      
      // Use speed of after segment if next curve small or
      // after curve tight
      if (s.after_rad)
	s_speed = speed_after;
      
      s_speed *= S_CURVE_MULT;
    }
  
  // printf("%lf %lf %lf\n", s.v, speed, speed_next);
  
  // If we are on a straightaway,
  if(s.cur_rad == 0.0)
    {
      crit = crit_dist(s.v, speed_next, -MAX_BRAKE_ACCEL);
      to_end = s.to_end;
      
      // Set course only shortly befor the next curve or if overtaking
      if (to_end <= PRE_STEER + crit*0.6)
	{
	  if (s.nex_rad < 0)
	    lane = 0.9 * width;
	  else
	    lane = 0.1 * width;
	  
	  // Steer
	  alpha = steer(s,CURVE_STEER, 1.8);
	}
      else if(s.dead_ahead)
	{
	  if(ovt_count == 0)
	    {
	      ovt_count = 250;
	      // Overtaking manuever, go to inside of next curve
	      if (lane > 0.8*width || lane < 0.2 * width)
		lane = 0.5 * width;
	      else if(s.nex_rad < 0)
		lane = 0.9 * width;
	      else
		lane = 0.1 * width;
	    }
	  // Steer
	  alpha = steer(s,0.6, 1.0);
	}
      else
	{
	  // If very fast, stay near the middle
	  if (s.v > 100.0 && ovt_count == 0 && to_end < 0.7 * s.cur_len)
	    if (s.nex_rad < 0)
	      lane = 0.2 * width;
	    else
	      lane = 0.8 * width;
	  
	  // Steer
	  alpha = steer(s,0.3, 1.0);
	}
      
      // Determine speed
      if (to_end > PRE_BRAKE + crit)
	vc = s.v + 50;
      else if (to_end > PRE_BRAKE)
	vc = s.v - MAX_BRAKE_ACCEL;
      else
	vc = wheel_speed(speed_next,s.v); // Softly enter the curve
      
      // Handle S-curve
      if (s_curve && s.seg_ID == s_segid && vc < wheel_speed(s_speed,s.v))
	vc = wheel_speed(s_speed,s.v);
      
    } // Straightway ends
  else
    { // Curve starts
      ovt_count = 0; // No overtaking in curves
      to_end = s.to_end * fabs(s.cur_rad);
      crit =crit_dist(s.v, speed_next, -CURVE_BRAKE_ACCEL);
      
      // Check if far enough around the corner
      curve_pos = (fabs(s.cur_len) - fabs(s.to_end)) / fabs(s.cur_len);
      if (curve_pos < CURVE_BRAKE)
	{
	  // Steer smothly around the corner
	  if (s.cur_rad > 0)
	    lane = 0.1 * width;
	  else
	    lane = 0.9 * width;
	  
	  alpha = steer(s,CURVE_STEER, 1.0);
	  
	  
	  vc = wheel_speed(speed,s.v);
	  
	  // Handle S-curve
	  if (s_curve)
	    vc = wheel_speed(s_speed,s.v);
	  
	}
      else // Second half of curve
	{
	  double corr = (curve_pos - CURVE_BRAKE) / CURVE_BRAKE;
	  
	  if (s.cur_rad > 0)
	    if (s.nex_rad > 0 || next_length < PRE_STEER)
	      lane = 0.1 * width; // Stay on the inside
	    else if (s.nex_rad != 0 && to_end <= PRE_STEER + crit*0.6 &&
		     cur_length > LONG_CURVE)
	      lane = (0.1 + 0.8 * corr)  * width; // Presteer into next curve
	    else
	      lane = (0.1 + 0.6 * corr)  * width; // Go to the middle
	  else
	    if (s.nex_rad < 0 || next_length < PRE_STEER)
	      lane = 0.9 * width; // Stay on the inside
	    else if (s.nex_rad != 0 && to_end <= PRE_STEER + crit*0.6 &&
		     cur_length > LONG_CURVE)
	      lane = (0.9 - 0.8 * corr) * width; // Presteer into next curve
	    else
	      lane = (0.9 - 0.6 * corr) * width; // Go to the middle
	  
	  // Prepare for S-Curves
	  if (s_curve && (s.seg_ID == s_segid || s.seg_ID == s_segid + 1))
	    {
	      if(s.nex_rad < 0)
		lane = (0.1 + 0.8 * corr) * width;
	      else
		lane = (0.9 - 0.8 * corr) * width;
	      alpha = steer(s,0.5, 1.0);
	    }
	  else
	    alpha = steer(s,CURVE_STEER, 1.0);
	  
	  // Default speed setting
	  if (speed > speed_next || fabs(s.to_end) > CURVE_ANGLE)
	    vc = wheel_speed(speed,s.v);
	  else
	    vc = wheel_speed(speed_next,s.v);
	  
	  
	  // If next curve is slower, hold old speed until brakeing point
	  if (s.v > speed_next && to_end < crit)
	    vc = wheel_speed(speed_next,s.v);
	  
	  // Handle S-curve
	  if (s_curve && (s.seg_ID == s_segid || s.seg_ID == s_segid + 1))
	    vc = wheel_speed(s_speed,s.v);
	}
      
    } // Curve

  //
  // Emergency brake (tm) section
  //
  
  // In any case, brake if necessary for next segments
  if (!(s_curve && s.seg_ID == s_segid) &&
      to_end + next_length < crit_dist(s.v, speed_after, -CURVE_BRAKE_ACCEL) &&
      s.v > speed_after)
    {
      // printf("Emergency brake\n");
      vc = wheel_speed(speed_after,s.v);
    }
  if (to_end + next_length + after_length <
      crit_dist(s.v, speed_aftaft, -CURVE_BRAKE_ACCEL) &&
      s.v > speed_aftaft)
    {
      // printf("Emergency brake 2\n");
      vc = wheel_speed(speed_aftaft,s.v);
    }

  // Avoid collusion with very slow cars (sigh)
  for (i = 0; i < 3; i++)
    {
      if (!pile_count &&                    // not already in avoidiance mode
	  s.nearby[i].who != 999 &&         // Valid car id
	  s.nearby[i].rel_ydot < -10.0 &&   // Closing in fast
	  // Car will cross my way when I reach it
	  //fabs(s.nearby[i].rel_x) <= 1.5 * CARWID &&
	   fabs(s.nearby[i].rel_x + (s.vn - s.nearby[i].rel_xdot) *
		s.nearby[i].rel_y /  s.nearby[i].rel_ydot) <= 2.0 * CARWID && 
	  // I have to brake to avoid it
	  s.nearby[i].rel_y - 1.5 * CARLEN< 
	  crit_dist(s.v, s.v + s.nearby[i].rel_ydot, -CURVE_BRAKE_ACCEL))
	{
	  // Slow down for a random time, or steer
	  pile_count = r_rand() * 5 / MAXRAND;
	  // Squeeze through attempt by random
	  if (s.nearby[i].rel_ydot < -10 *(r_rand() * slow_down * 5 / MAXRAND))
	      pile_action = (int) (r_rand() * slow_down * 1.5 / MAXRAND);
	  // printf("Emergency brake 3  for %d - length %d\n",i,pile_count);
	  break;
	}
    }

  // Dont get caught in pileups
  for (i = 0; i < 3; i++)
    {
      if (!pile_count && !ovt_count &&       // not already in avoidiance mode
	  s.nearby[i].who != 999 &&          // Valid car id
	  fabs(s.nearby[i].rel_x) <= 1.5 * CARWID && // I'm stuck within
	  s.nearby[i].rel_y <= 1.5 * CARLEN)
	{
	  // Slow down for a random time
	  pile_count = r_rand() * 5 / MAXRAND;
	  pile_action = 0;
	  // printf("Emergency brake 4  for %d - length %d\n",i,pile_count);
	  
	  break;
	}
    }

  if (pile_count)
    {
      if (!pile_action)
	{
	  // Reduce speed, and steer away from border
	  vc = s.v - CURVE_BRAKE_ACCEL;
	  if (vc < 10.0)
	    vc = 10.0;
	  if (!ovt_count)
	    {
	      lane = 0.5 * width;
	      alpha = steer(s,CURVE_STEER, 1.0);
	    }
	  pile_count--;
	}
      else
	{
	  // No action, just squeeze through
	}
    }

  // Avoid collusions when refueling
  if (s.fuel <= 0.01)
    {
      static int last_fuel;
      // if (s.fuel >= 0.0) fprintf(stderr,"%lf\n",s.fuel);
      if (last_fuel)
	{
	  vc = wheel_speed(30.0, s.v);
	  last_fuel = 0;
	}
      else
	{
	  vc = wheel_speed(10.0, s.v);
	  last_fuel = 1;
	}
	
    }

  //
  // End of emergency brake (tm) section
  //

      
  // printf("%lf %lf %lf\n",s.v,vc, speed);
  
  //if (s_curve)
  //  printf("%lf %lf %lf\n", s_speed, vc, s.v);

  
  //
  // Custom code ends
  //
  result.vc = vc;   result.alpha = alpha;

if(s.starting)  result.fuel_amount = MAX_FUEL;
result.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
  result.request_pit = 1;
  result.repair_amount=s.damage;
  result.fuel_amount = MAX_FUEL;
  }
  return result;
}

