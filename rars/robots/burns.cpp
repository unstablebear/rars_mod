// BURNS.CPP - "driver" function for RARS v0.50 
// by Safai Ma, April 95, bg174@freenet.toronto.on.ca
// This robot driver is very simple minded.  He will stay in the middle of
// the track when he is on the straights and stay in the inside when
// cornering. No code is done to handle any traffic though.  Whenever I
// try to do it, it just slows him down :(  With this set of hand-optimized
// parameters set, speed up is already possible by tweaking the parameters set.
// (track specific parameter data files is not implemented yet :( this could
// really speed he up)  Right now, he will stays on all the tracks that are
// available on the ftp site. :)  Have fun!

//////////////////////////////  I N C L U D E S  ///////////////////////////////

#include <string.h>                                                // strcpy
#include <math.h>                                                  // fabs, sqrt

#include "car.h"                                                   // stuck

///////////////////////////////  D E F I N E S  ////////////////////////////////

#define ACCEL                   35.00     // acceleration
#define BIAS                     0.06     // estimate of alpha when cornering
#define BIG_SLIP                17.00     // affects the bias of steering servo
#define CORN_SPD_CON             5.86     // determines how fast to take corners
#define DECEL                   35.00     // braking deceleration
#define DELTA_LANE              10.00     // when dead ahead, change lane
#define END_CR                   1.40     // end of corners
#define END_ST                   2.70     // end of straights
#define MIN_CORN_SPD            52.00     // minimum cornering speed
#define SHORT_CORN               0.60     // fraction of width considered short
#define SHORT_CORN_SPD           5.60     // short corner speed constant
#define SMALL_CURVE              1.10     // small curvature (rad/len)
#define START_ACC                0.55     // fraction of seg. len. to start acc.
#define STEER_DAMP               1.50     // servo damping, to prevent "weaving"
#define STEER_GAIN               2.50     // servo gain, for staying in "lane"
#define STRAIGHT_SPD           400.00     // speed for straights

///////////////////////////////  E X T E R N S  ////////////////////////////////


////////////////////////////////  M A C R O S  /////////////////////////////////

#define max(x,y)                (((x)<(y))?(y):(x))            // return maximum
#define min(x,y)                (((x)>(y))?(y):(x))            // return minimum
#define sqr(x)                  ((x)*(x))                      // return x}

/////////////////////  S U P P O R T   F U N C T I O N S  //////////////////////

////                                                                        ////
 //     calculate cornering speed (funny name to avoid duplications)         //
////                                                                        ////

static double burns_cs (double radius, double len, double lane, double width)
{
    double s;

    if (radius < 0.0)                     // change sign of negative radius
    {
        radius = - radius;                // make it positive
        lane = width - lane;              // get the right lane for right turn
    }
    else if (radius == 0.0)               // return a high speed for straights
        return (STRAIGHT_SPD);

    // calculate cornering speed
    s = max (MIN_CORN_SPD, CORN_SPD_CON * sqrt (radius + lane));

    if (radius/len > SMALL_CURVE)         // for small curvature, increase speed
        s += CORN_SPD_CON * sqrt (SHORT_CORN_SPD * radius / len);
    else if (len < SHORT_CORN * width)    // for short corners, increase speed
        s += CORN_SPD_CON * sqrt (SHORT_CORN_SPD * 7.0);

    return s;
}                                                     // end of routine burns_cs

////                                                                        ////
 //     calculate segment length (for both curves and straights), in feet    //
////                                                                        ////

static double burns_sl (double rad, double len, double width)
{
    if (rad != 0.0) return ((fabs (rad) + 0.5 * width) * len);
    return (len);
}                                                     // end of routine burns_sl

////                                                                        ////
 //     decides which lane to drive on (lane selection, wrt to left)         //
////                                                                        ////

static double burns_ls (double rad, double width, double len)
{
    double lane = .5 * width;

    if (rad == 0.0) lane = .5 * width;
    else if (len < .5 * width) lane = (rad > 0.0 ? .4 : .6) * width;
    else if (rad < 0.0) lane = ((width < 100) ? (.75 * width) : (width - 25));
    else if (rad > 0.0) lane = ((width < 100) ? (.25 * width) : (25));

    return lane;
}                                                     // end of routine burns_ls

////                                                                        ////
 //     calculate braking distance using current and goal speed              //
////                                                                        ////

static double burns_bd (double cur_v, double nex_v)
{                                    // return -ve value if no braking is needed
    return ((sqr(cur_v) - sqr(nex_v))/(2*DECEL));
}                                                     // end of routine burns_bd

///////////////////////  D R I V E R   F U N C T I O N  ////////////////////////

con_vec Burns (situation& s)
{
    const char name[] = "Burns";           // This is the robot driver's name!
    static int init_flag = 1;              // cleared by first call
    double alpha, vc;                      // components of result
    con_vec result;                        // This is what is returned

    if (init_flag)                         // for first time only
    {
        my_name_is(name);          // copy name over
        init_flag = 0;                     // reset init flag
        result.alpha = result.vc = 0;
        return result;
    }

    // get the car out of an abnormal condition, thanks Mitchell :)
    if (stuck (s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc))
        return result;

    // get track width
    double width = s.to_lft + s.to_rgt;

    // convert s.to_end to feets (for curves, don't want to work with radians :)
    double to_end = burns_sl (s.cur_rad, s.to_end, width);

    // convert s.cur_len, s.nex_len to feets (for curves)
    double cur_len = burns_sl (s.cur_rad, s.cur_len, width);
    double nex_len = burns_sl (s.nex_rad, s.nex_len, width);

    // don't really need to convert this one but...just to make it consistent
    // we don't have s.after_len :)
    double aft_len = burns_sl (s.after_rad, s.after_rad, width);

    // select a lane for the current and the coming two track segments
    double cur_lane = burns_ls (s.cur_rad, width, cur_len);
    double nex_lane = burns_ls (s.nex_rad, width, nex_len);
    double aft_lane = burns_ls (s.after_rad, width, aft_len);

    // convert a lane value to alpha (a standard procedure)
    double cur_alpha = STEER_GAIN * (s.to_lft - cur_lane) / width;
    double nex_alpha = STEER_GAIN * (s.to_lft - nex_lane) / width;

    // calculate intermediate alpha when changing lanes
    double ratio = ((s.cur_rad == 0.0) ? END_ST : END_CR) * width;
    if (ratio > cur_len) ratio = cur_len;
    if (to_end < ratio)
        alpha = cur_alpha*to_end/ratio + nex_alpha*(1-to_end/ratio);
    else
        alpha = cur_alpha;

    // calculate target speeds for the three track segments
    double speed       = burns_cs (s.cur_rad,   cur_len, cur_lane, width);
    double speed_next  = burns_cs (s.nex_rad,   nex_len, nex_lane, width);
    double speed_after = burns_cs (s.after_rad, aft_len, aft_lane, width);

    // calculate braking distance for the next two track segments
    double cur_brake_dist = burns_bd (s.v, speed_next);
    double nex_brake_dist = burns_bd (s.v, speed_after);

    // now set the tire speed, vc
    if ((to_end + nex_len) < nex_brake_dist)      // brake if neccessary
        vc = speed_after;                         // for the 'after' corner
    else if (to_end < cur_brake_dist)
        vc = speed_next;                          // brake for next corner
    else if ((to_end < (START_ACC * cur_len)) && (s.v < speed_next))
        vc = speed_next;                          // accelerate for next segment
    else
        vc = speed;                               // keep current speed

    // calculate the bias (bias must be negative for right turn)
    double bias = ((s.cur_rad!=0.0)?(sqr(s.v)/sqr(speed))*atan(BIG_SLIP/speed):0.0);
    if (s.cur_rad < 0.0) bias = - bias;

    // adding bias and damping to prevent oscillation
    alpha += bias - STEER_DAMP * s.vn / s.v ;

    // return calculated vc and alpha to host program
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

//////////////////////  E N D   O F   B U R N S . C P P  ///////////////////////


