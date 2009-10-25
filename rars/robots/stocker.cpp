// STOCKER.CPP - A robot "driver" for RARS, by M. Timin, August, 1955
// for ver. 0.63b

#include <stdlib.h>
#include <math.h>
#include "car.h"

const double CORN_SPD_CON = 5.75; // determines how fast to take corners
const double BIAS = 0.05;         // estimate of alpha when cornering
const double STEER_GAIN = 0.7; // servo gain, for staying in "lane"
const double STEER_DAMP = 2.0;    // servo damping, to prevent "weaving"
const double SLIP_LIM = 4.0;     // maximum wheel slip, ft/sec, in sdw()
const double SLIP_CON = 250.0;   // determines wheel slip during dragout
const double SHARP_TURN = .1;    // change in alpha when attempting to pass
const int PASSING_TIME = 70;     // time to stay in passing maneuver, counts
const double CUSHION = 10.0;     // feet from inside rail to shoot for
const double CRIT_WIDTHS = 2.9;  // length in widths meaning nearing corner
const double BRAK_ACCEL = 23.0;  // braking acceleration, ft/sec sq.
const double MARGIN = 9.5;     // target distance from curve's inner rail
const double BRK_CRV_SLIP = 4.0;  // tire slip for braking in curve
const double DELTA_LANE = 2.3;   // if collision predicted, change lane by this

static double sdc(double radius)  // SpeeD Cornering
{                                 // calculates a cornering speed
   if(radius < 0.0)         // change sign of negative radius
      radius = -radius;
   else if(radius == 0.0)   // This is just insurance, this funtion doesn't
      return(200.0);        // make sense when the radius is zero.
   return CORN_SPD_CON * sqrt(radius);
}

static double sdw(double goal, double present)  // SpeeD Wheel
{                                        // Calculates a tire speed
   double ws;

   if(present > goal + 2 * SLIP_LIM)  // if too fast,
      ws = present - SLIP_LIM;      // slow down.
   else if(present < goal - 2 * SLIP_LIM)  // if too slow,
      ws = present + SLIP_LIM;             // accelerate.
   else                           // if quite close,
      ws = (goal + present) / 2;      // approach desired speed gently.

   if(ws < present)          // can slip more when braking
     ws -= (present - ws);

   return ws;
}

static double widths(double len, double rad, double wide)
{                      // computes a distance in track widths
   if(rad == 0.0)
      return len/wide;
   else if(rad > 0.0)
      return (rad+.5*wide)*len/wide;
   else
      return (.5*wide - rad)*len/wide;
}

con_vec Stocker(situation &s)
{
   const char name[] = "Stocker";      // This is the robot driver's name!
   static int init_flag = 1;          // cleared by first call
   double speed;                      // target speed for cornering, ft/sec
   double speed_next;                 // target speed for next corner
   con_vec result;                    // This is what is returned.
   double width;                      // track width, feet
   double alpha, vc;           // components of result
   double dest;                // target distance from left wall
   double arg;                 // for temporary storage of calculation
   double widths_cur;          // length remaining of current segment, widths
   static int started = 0;     // set after the "dragout"
   double redline;             // speed at which to begin braking on straight
   double bias;                // estimated alpha value for a curve
   static double lane_inc = 0.0;  // an adjustment to "lane", for passing

   if(init_flag)  {            // first time through, only copy name:
      my_name_is(name);
      init_flag = 0;
      result.alpha = result.vc = 0;
      return result;
   }
  if(s.starting) result.fuel_amount = MAX_FUEL;
  if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
      return result;

   width = s.to_lft + s.to_rgt;
   widths_cur = widths(s.to_end, s.cur_rad, width);

   if(s.cur_rad == 0.0)
      if(s.nex_rad > 0.0)
         dest = 2.0 * CUSHION;
      else
         dest = width - 2.0 * CUSHION;
   else if(s.cur_rad > 0.0)
      if(s.nex_rad >= 0.0)
         dest = CUSHION;
      else if(widths_cur > CRIT_WIDTHS)
         dest = CUSHION;
      else
         dest = .5 * width;
   else
      if(s.nex_rad <= 0.0)
         dest = width - CUSHION;
      else if(widths_cur > CRIT_WIDTHS)
         dest = width - CUSHION;
      else
         dest = .5 * width;

   // calculate target speeds for current corner and the next:
   if(s.cur_rad == 0.0)
      arg = 0.0;
   else if(s.cur_rad > 0.0)
      arg = s.cur_rad + .5 * width;
   else
      arg = s.cur_rad - .5 * width;
   speed = sdc(arg);      // speed is based on radius

   if(s.cur_rad == 0.0)     // calculate a bias for alpha in a turn:
      bias = 0.0;
   else if(s.cur_rad > 0.0)
      bias = 100 * BIAS / speed;
   else
      bias = -100.0 * BIAS / speed;

   if(s.nex_rad == 0.0)     // calculate speed for next turn
      arg = 0.0;
   else if(s.nex_rad > 0.0)
      arg = s.nex_rad + .5 * width;
   else
      arg = s.nex_rad - .5 * width;
   speed_next = sdc(arg); // of center line of track.

   alpha = bias + STEER_GAIN * (s.to_lft - dest) / width;
   alpha -= STEER_DAMP * s.vn / s.v;  // This is damping, to prevent oscillation

   // now set the tire speed, vc:
   if(s.cur_rad == 0.0) {        // If we are on a straightaway,
      redline = sqrt(speed_next * speed_next + BRAK_ACCEL * 2.0 * s.to_end);
      if(s.v > .85 * redline)
          started = 1;
      if(s.v < redline)
         vc = s.v + SLIP_CON / s.v;     // keep accellerating near full power
      else                     // otherwise,
         vc = sdw(speed_next, s.v);    // brake for next corner
   }
   else                              // If we're in the curve, maintain speed.
      if(widths_cur > CRIT_WIDTHS)
                  // if we are far from the next corner, stay at "speed".
         vc = sdw(speed, s.v);
      else        // but when we near the next corner, adjust to "speed_next"
         vc = sdw(speed_next, s.v);
   if(!started)   // don't steer during dragout
      if(s.vn > .02 * s.v || s.vn < -.02 * s.v)    // except if astray
         started = 1;
      else
         alpha = 0;

   // Passing and anti-collision code:
   // This code first tries to predict a collision; if no collision is
   // predicted, it does nothing.  Collision prediction is approximate, and
   // is based on linear extrapolation.  This can work because it is
   // repeated eighteen times per second of simulated time.
   // If a collision is predicted, then it gradually changes the
   // lane_inc static variable which changes alpha.
   // The hope is to steer around the car.  When no collision is
   // predicted then lane_inc is gradually brought back to zero.
   // If a crash is about to occur, medium hard braking occurs.
   double x, y, vx, vy, dot, vsqr, c_time, y_close, x_close;
   int kount;     // counts cars that are in danger of collision
   kount = 0;
   for(int i=0;i<3;i++) if (s.nearby[i].who<16) { // if there is a close car
      y=s.nearby[i].rel_y;         // get forward distance (center-to-center)
      x=s.nearby[i].rel_x;         // get right distance
      vx=s.nearby[i].rel_xdot;     // get forward relative speed
      vy=s.nearby[i].rel_ydot;     // get lateral relative speed
      // if the cars are getting closer, then the dot product of the relative
      // position and velocity vectors will be negative.
      dot = x * vx + y * vy;     // compute dot product of vectors
      if(dot > -0.1)            // no action if car is not approaching.
         continue;
      vsqr = vx*vx + vy*vy;      // compute relative speed squared
      // Time to closest approach is dot product divided by speed squared:
      c_time = -dot / vsqr;     // compute time to closest approach
      if(c_time > 3.0)          // ignore if over three seconds
         continue;
      /* If the execution gets this far, it means that there is a car
      ahead of you, and getting closer, and less than 3.0 seconds
      away.  Evaluate the situation more carefully to decide if
      evasive action is warranted: */
      x_close = x + c_time * vx;      // x coord at closest approach
      y_close = y + c_time * vy;      // y coord at closest approach
      /*  Due to the length of the cars, a collision will occur if
          x changes sign while y is less than CARLEN.  This
          can happen before the center-to-center distance reaches its
          point of closest approach. */
      // check if collision would occur prior to closest approach
      // if so, reduce c_time, re-calculate x_close and y_close:
      if(x_close * x < 0.0 && y < 1.1 * CARLEN) {
         c_time = (fabs(x) - CARWID) / fabs(vx);
         x_close = x + c_time * vx;      // x coord at closest approach
         y_close = y + c_time * vy;      // y coord at closest approach
      }
      // Will it be a hit or a miss?
      if(fabs(x_close) > 2 * CARWID || fabs(y_close) > 1.25 * CARLEN)
         continue;            // this when a miss is predicted
      // If we get here there is a collision predicted
      ++kount;    // This counts how many cars are in the way.
      if(kount > 1 || c_time < .85)  // if more than one problem car, or if
         vc = s.v - BRK_CRV_SLIP;    // car within .85 sec of collision, brake!
      // steer to avoid the other car:
      // if there is room, we try to pass with least x deviation
      if(s.cur_rad > 0.0)
         if(x_close < 0.0 || s.to_lft < MARGIN)  // avoid scraping the inside
            lane_inc += DELTA_LANE;
         else
            lane_inc -= DELTA_LANE;
      else if(s.cur_rad < 0.0)
         if(x_close > 0.0 || s.to_rgt < MARGIN)
            lane_inc -= DELTA_LANE;
         else
            lane_inc += DELTA_LANE;
      else if(x_close < 0.0)      // on straights, pass with least x deviation
         lane_inc += DELTA_LANE;
      else
         lane_inc -= DELTA_LANE;
      if(lane_inc > .25 * width)  // limit the lane alteration to 1/4 width:
         lane_inc = .25 * width;
      else if(lane_inc < -.25 * width)
         lane_inc = -.25 * width;
    }

    // Here we gradually reduce lane_inc to zero if no collision is predicted:
    if(!kount)
      if(lane_inc > .1)
         lane_inc -= .5*DELTA_LANE;
      else if(lane_inc < -.001)
         lane_inc += .5*DELTA_LANE;

    // lane_inc represents an adjustment to the lane variable.  This is
    // accomplished by changing alpha an amount equal to that which the
    // steering servo would have done had lane actually been changed.
    result.vc = vc;   result.alpha = alpha - STEER_GAIN * lane_inc / width;
    result.request_pit=0;
    if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
      {
	result.request_pit=1;
	result.repair_amount=s.damage;
	result.fuel_amount=MAX_FUEL;
      }
    return result;
}


