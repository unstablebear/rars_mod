/* osccar2.cpp Oscar Gustafsson o__g@spray.se OscCar2
   All races during the 2001 season
   Not confidentail source
   No data files */

#include <fstream.h>
#include <iostream.h>
#include <iomanip.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"

const double    KURV_FART_KONST = 6;
const double    KURV_BROMS = -24.0;
const double    RAK_BROMS = -28.0;
const double    RAK_BROMS_KVOT = .91;
const double    RAK_BROMS_KONST = 10;
const double    BIG_SLIP = 15.0;
const double    STEER_GAIN = .7;
const double    DAMP_GAIN = 2.0;
const double    DELTA_LANE = 1.5;
const double    WANTED_DIST = 10.0;
const double    BIG_RAD = 3.0;
const double    HS_RAD = 500;

//ofstream filut("osccar2c.dat");

double          absolut(double x)
{
    if (x < 0)
    return -x;
    else
    return x;
}


double          kurv_fart(double radius)
{
    double       speed;
    if (!radius)
    return 250.0;
    speed = (KURV_FART_KONST * sqrt(absolut(radius)));
    if(speed < 60)
        return 60;
    else
        return speed;

}

double          kurv_kaning(double radius)
{
    if (absolut(radius) > HS_RAD)
    return 10.0;
    else
    return 5.5;
}

double          krit_avst(double v0, double v1, double a)
{
    double          dv;
    double          utv;
    dv = v1 - v0;
    if (dv > 0.0)
    return 0.0;
    utv = (v0 + .5 * dv) * dv / a;
    return utv;
}

con_vec         OscCar2(situation& s)
{
    int             brake = 0;
    const char      name[] = "OscCar2";
    static int      init_flag = 1, high_speed=0;
    con_vec         result;
    double          alpha,
            vc;
    static double   lane = -10000;
    static double   speed_next = 250;
    static double   speed_goal = 250;
    double  lane_inc = 0,
            bias,
            width,
            speed_after;
    static double   prev_rad = 10000.0, prev_len = 10000.0;

    if (init_flag == 1) {
    my_name_is(name);
    init_flag = 0;
    result.alpha = result.vc = 0;
    return result;
    }
    if (stuck(s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc)) {
    return result;
    }
    width = s.to_lft + s.to_rgt;

    if (lane < -9000)
    lane = s.to_lft;

    if(absolut(s.nex_rad)<HS_RAD && s.nex_len<1.2 && s.nex_rad != 0 && ((s.cur_rad == 0) 
||
        absolut(s.cur_rad)>HS_RAD) && (absolut(s.after_rad)>HS_RAD ||
        s.after_rad == 0))
    {
//             filut << "Picaboo" << endl;
        speed_next = kurv_fart(absolut(s.nex_rad)+(width-WANTED_DIST)/(1-cos(s.nex_len)));
 //            filut << "   " << speed_next << endl;
        high_speed = 2;
    }

    if(prev_len != s.cur_len || prev_rad != s.cur_rad)
    {
        prev_rad = s.cur_rad;
        prev_len = s.cur_len;
        if(high_speed)
            high_speed--;
    }


    if (s.cur_rad == 0) {
    bias = 0.0;
    speed_goal = s.v + 50;
    if (s.nex_rad == 0)
        speed_next = 250;
    else {
        if(!high_speed)
            speed_next = kurv_fart(absolut(s.nex_rad) + WANTED_DIST);
        if (s.nex_rad < 0)
                lane = width - WANTED_DIST;
            else
                lane = WANTED_DIST;
        }
    }
    else
    {
    if ((s.nex_rad * s.cur_rad < 0) || (s.nex_rad == 0 && s.after_rad * s.cur_rad < 0)) {
        if (s.to_end < s.cur_len / 2)
        lane = width / 2;
        if (s.to_end < s.cur_len / 6)
        if (s.cur_rad < 0)
            lane = WANTED_DIST;
        else
            lane = width - WANTED_DIST;
    }
    else
    {
        if (s.cur_rad < 0)
        lane = width - WANTED_DIST;
        else
        lane = WANTED_DIST;
    }


    if(high_speed==1)
        speed_goal = kurv_fart(absolut(s.cur_rad)+width/(1-cos(s.cur_len)));
    else
    {
    if (s.cur_rad < 0)
        speed_goal = kurv_fart(width - lane - s.cur_rad);
    else
        speed_goal = kurv_fart(s.cur_rad + lane);

    if (s.cur_len > BIG_RAD && s.to_end < s.cur_len)
        speed_goal = kurv_fart(absolut(s.cur_rad) + width / 2);
    }

    bias = (s.v * s.v / (speed_goal * speed_goal)) * atan(BIG_SLIP / speed_goal);
    if (s.cur_rad < 0.0)
        bias = -bias;

    if(high_speed != 2)
        if (s.nex_rad == 0)
        speed_next = 250;
        else if (s.nex_rad < 0)
        speed_next = kurv_fart(WANTED_DIST - s.nex_rad);
    else
        speed_next = kurv_fart(s.nex_rad + WANTED_DIST);

    }

    alpha = STEER_GAIN * (s.to_lft - lane) / width - DAMP_GAIN * s.vn / s.v + bias;

    if (absolut(alpha) > 1.6)
    alpha = s.cur_rad < 0 ? -1.6 : 1.6;

    if (s.cur_rad == 0) {
    if (s.to_end < krit_avst(s.v, speed_next, RAK_BROMS)) {
        brake = 1;
    }
    } else if (s.to_end * (absolut(s.cur_rad) + lane) <= krit_avst(s.v, speed_next, 
KURV_BROMS)) {
    brake = 1;
    }
    if (s.after_rad != 0) {
    speed_after = kurv_fart(absolut(s.after_rad) + WANTED_DIST);
    if (s.cur_rad == 0) {
        if (s.to_end + s.nex_len * (absolut(s.nex_rad) + WANTED_DIST)
        <= krit_avst(s.v, speed_after, KURV_BROMS)) {
        brake = 1;
        }
    } else if (s.nex_rad == 0) {
        if (s.to_end * (absolut(s.cur_rad) + lane) + s.nex_len
        <= krit_avst(s.v, speed_after, KURV_BROMS)) {
        brake = 1;
        }
    } else {
        if (s.to_end * (absolut(s.cur_rad) + lane) + s.nex_len * (absolut(s.nex_rad) + 
WANTED_DIST)
        <= krit_avst(s.v, speed_after, KURV_BROMS)) {
        brake = 1;
        }
    }
    }
    if (brake)
    if (s.cur_rad == 0)
    {
        if (s.v > 1.02 * speed_next)
        {
        vc = s.v - RAK_BROMS_KONST;
        }
        else if (s.v < 0.98 * speed_next)
            vc = 1.1 * speed_next;
        else
            vc = 0.5 * (s.v + speed_next);
    }
    else
    {
        vc = s.v - kurv_kaning(s.cur_rad);
    }
    else if (s.cur_rad == 0)
    vc = s.v + 50;
    else
    vc = .5 * (s.v + speed_goal) / cos(alpha);

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
         vc = s.v - kurv_kaning(s.cur_rad);    // car within .85 sec of collision, brake!
      // steer to avoid the other car:
      // if there is room, we try to pass with least x deviation
      if(s.cur_rad > 0.0)
         if(x_close < 0.0 || s.to_lft < WANTED_DIST)  // avoid scraping the inside
            lane_inc += DELTA_LANE;
         else
            lane_inc -= DELTA_LANE;
      else if(s.cur_rad < 0.0)
         if(x_close > 0.0 || s.to_rgt < WANTED_DIST)
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

//      filut << "Alpha: " << alpha << " Vc: " << vc << " Radie: " <<  s.cur_rad <<" Vg: " <<  
speed_goal;
//      filut << " Vn: " << speed_next<< " V: " << s.v << " Hs: " << high_speed << endl;

    result.vc = vc;
    result.alpha = alpha;
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


