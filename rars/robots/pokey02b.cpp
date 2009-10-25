//      Drive Name: Pokey02
// Filename: Pokey02.cpp
// Date : 25 May 1998
// Author: Stuart Strickland
// Email: strick@merle.acns.nwu.edu
//
// Race: 28. May 1998 BORS Races (Indy & Pocono)
//

#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "track.h"

#define min(x,y)        (((x)>(y))?(y):(x))
#define max(x,y)        (((x)>(y))?(x):(y))
#define sqr(x)          ((x)*(x))

//
//              STRUCTURES
//

static struct corn_info {double dive, add_r;}ci;

//
//              CONSTANTS
//

const double MAX_SPD = 400.0;
const double FAST_CORNER = 180.0;

const double BRK_SLIP_LIM = 38;
const double ACC_SLIP_LIM = 38;
const double ABOVE_SPD = 12;
const double BELOW_SPD = 8;
const double BIG_SLIP = 16.0;


const double DRAG_GAIN = 0.5;
const double DRAG_DAMP = 0.6;

const double STRT_GAIN = 0.45;
const double STRT_DAMP = 0.50;

const double CORN_GAIN = 0.65;
const double CORN_DAMP = 0.52;

const double CONTROL_TURN = 0.90;
const double CONTROL_EXIT = 1.04;


//
//              TRACK-DEPENDENT GLOBAL VARIABLES
//

static double TURN_BRK_SLIP_LIM;
static double WIDTH;
static int DRAG_TIME;
static double IN_BUFF;
static double OUT_BUFF;
static double SAFE_BUFF;
static double DEPTH;
static double EXIT;
static double DRIFT;
static double CORN_CON;
static double PIT_BUFF;
static double TURN_AGG;
static double STRT_AGG;

static track_desc track;


//
//              FUNCTIONS
//

static void track_init(void)
{
	track = get_track_description();
}

static double divide(double n, double d)
{
	if ((d >= 0.0) && (d < .01)) d = .01;
   if ((d < 0.0) && (d > -.01)) d = -.01;
   return n/d;
}

static double corner_spd(double radius)
{
	double speed;
   if (radius == 0.0)
	return(MAX_SPD);
   if (radius < 0.0) radius = -radius;
   speed = CORN_CON * sqrt(radius);
   return speed;
}

static double lane_glide(double tot_dist, double rem_dist, double start_lane, double end_lane)
{
	double fraction_done, lane_delta;

   fraction_done = divide ((tot_dist - rem_dist), tot_dist);
   lane_delta = end_lane - start_lane;

   return start_lane + (fraction_done * lane_delta);
}

static double calc_vc(double target, double current, double brake_lim)
{
	if (current > target + ABOVE_SPD)
	return (current - brake_lim);
   if (current < target - BELOW_SPD)
	return (current + ACC_SLIP_LIM);
   return (divide(((2 * target) + current), 3));
}

static double calc_corn0(double radius, double width)
{
	double speed;
   ci.add_r = 0;
   ci.dive = 0;

   if (radius < 0.0) radius = -radius;
   radius += width;
   speed = CORN_CON * sqrt(radius);

   return speed;
}

static double brak_dist(double start, double end, double brake)
{
	double delta;

   if (start <= end)
	return 0.0;
   delta = start - end;

   return (start - (divide(delta,2))) * divide(delta,brake);
}

static void calc_corn1(double theta, double wide)
{
	if (theta > PI/2) theta = PI/2;
   ci.add_r = divide(wide,1-cos(theta));
   ci.dive = ci.add_r * sin(theta);
}
static double distancetopass(double vnow, double vthen, double radius)
{
  if (vnow<vthen)
    return 0.0;
  else if (radius != 0)
    return (TURN_AGG * (vnow+vthen) * (vnow-vthen));
  return (STRT_AGG * (vnow+vthen) * (vnow-vthen));

}

//
//              COLLISION AVOIDANCE
//

static void avoid_cars(situation &s, con_vec &result)
{
  int danger = 0;
  int kount = 0;
  double vc1;
  double v[4], d[4], x[4], y[4], vx[4], vy[4];
  for(int i=0;i<4;i++)
    if(s.nearby[i].who<25)
      {
	x[i]=s.nearby[i].rel_x;         // distance to right (or left if < 0)
	y[i]=s.nearby[i].rel_y;         // distance ahead, always positive
	vx[i]=s.nearby[i].rel_xdot;     // relative lateral speed component
	vy[i]=s.nearby[i].rel_ydot;     // relative forward speed (negative)
	d[i]=sqrt(x[i]*x[i] + y[i]*y[i]);       // distance to other car
	v[i]=sqrt(vx[i]*vx[i] + vy[i]*vy[i]);   // relative speed

	if (vy[i] > -1)  continue;      // ignore faster cars
		else if (d[i] > 1.2*distancetopass(s.v, s.v+vy[i]+5,s.cur_rad))
	  continue;       // check whether you need to brake
	++kount;
	if (fabs(x[i])>1.5*CARWID && x[i]*vx[i]>-10 &&
	    fabs(vx[i])>10 && vy[i]>-10)
	  continue; // if slow car ahead, reserve more safety space
	else if (fabs(x[i])>CARWID && x[i]*vx[i]>-10 &&
		 fabs(vx[i])>10 && vy[i]<=-10)
	  continue; // if fast car ahead, reserve less safety space
	else danger = 1;// brake or pass if trouble car within braking distance
}

//
//              Pass or brake?
//

  if (danger){
    if(kount > 2 || s.damage > 25000)
      vc1 = min((s.v + vy[0] + 5),(s.v + vy[1] + 5));
    else{
      vc1 = min((s.v + vy[0] + 5),((s.damage >20000?0.9:0.95)*result.vc));
      if (!s.cur_rad){  //straigth => pass!
	if (x[0] < 0)  // car in left
	  if (s.to_lft + x[0] < track.width - 1.5* CARWID)
	    result.alpha -= 0.03;
	//if we have room in right, turn right
	  else    result.alpha += 0.03;
	else if (x[0] >= 0)    // car in right
	  if (s.to_lft + x[0] > 1.5* CARWID) // if we have room in left
	    result.alpha += 0.03; //turn left
	  else  result.alpha -= 0.03;
      }
    } // end of kount < 2
  }// end of danger
  else vc1 = result.vc; //this executes if no danger seen

  vc1 = max(vc1,5);  // To avoid stopping behind very slow car
  result.vc = vc1;
} // end of avoid_cars function



//
//              MAIN PROGRAM
//

con_vec Pokey02(situation &s)
{
	const char name[] = "Pokey02";

   static int init_flag = 1;
   static int strting =1;
   static int start_timer =0;
   static int last_seg = -99;
   static double lane;
   static double togo_start;
   static double next_wide_aim;
   static double next_wide_start;
   static double next_in_aim;
   static double next_r;
   static double next_cd;
   static double next_cs;
   static double this_wide_aim;
   static double this_wide_start;
   static double this_in_aim;
   static double this_cd;
   static double this_cs;
   static int double_corner;
   static int pit_exit = 0;

   con_vec result;

   double alpha, vc, speed;
   double theta, bdst, to_apex, brake_lim, lane_gonna;
   double steer_gain, damp_gain, bias;

   if(init_flag)        {
	my_name_is(name);
      init_flag = 0;
      result.alpha = 0;
      result.vc = 0;
      return result;
   }

   //
   //           STARTING SITUATION (ASSIGN VALUES TO TRACK-DEPENDENT GLOBAL VARIABLES)
   //

   if(strting)  {

	track_init();

	WIDTH = (s.to_lft + s.to_rgt);
      OUT_BUFF = (WIDTH-55);
		TURN_BRK_SLIP_LIM = 20.0;
      DRAG_TIME = 100;
      CORN_CON = 5.8;
      SAFE_BUFF = 12.0;
      IN_BUFF = 4.0;
      DEPTH = 42.0;
      EXIT = PI * .36;
      DRIFT = 2.5;
      PIT_BUFF=10;
      TURN_AGG=0.11;
      STRT_AGG=0.011;

      if(s.stage == QUALIFYING) {
	result.fuel_amount = 12*track.length*.0003;
      }
      else result.fuel_amount = MAX_FUEL;


	if(strting == 1)        {
	lane = s.to_lft;
	 if(s.to_rgt < OUT_BUFF)
		lane = WIDTH - OUT_BUFF;
	 strting = 2;
      }
      if (start_timer++ >= DRAG_TIME)
	strting =0;
      result.vc = MAX_SPD;
      result.alpha = DRAG_GAIN * (divide((s.to_lft - lane),WIDTH)) - DRAG_DAMP * divide(s.vn,s.v);
      return result;
   }

   //
   //           NORMAL RACING SITUATION
   //

  if (s.seg_ID != last_seg) {
    double_corner = 0;
    last_seg = s.seg_ID;
    next_wide_start = s.to_lft;
    togo_start = s.to_end;

    if (s.cur_rad == 0.0) {
      if (s.nex_rad > 0.0) {
	theta = s.nex_len / 2;
	lane_gonna = s.to_lft;
	if (lane_gonna < SAFE_BUFF) lane_gonna = SAFE_BUFF;
	if (lane_gonna > WIDTH - SAFE_BUFF) lane_gonna = WIDTH-SAFE_BUFF;
	if (calc_corn0(s.nex_rad,lane_gonna) > FAST_CORNER) {
	  next_wide_aim = lane_gonna;
	  next_in_aim = lane_gonna;
	  next_r = s.nex_rad + lane_gonna;
	  next_cs = calc_corn0(s.nex_rad,lane_gonna);
	  next_cd = next_r * theta;
	}
	else {
	  calc_corn1(theta, WIDTH-OUT_BUFF);
	  next_wide_aim = WIDTH-OUT_BUFF;
	  next_in_aim = IN_BUFF;
	  next_r = s.nex_rad + ci.add_r;
	  next_cs = corner_spd(next_r);
	  next_cd = next_r * theta;
	}
      }
    }
    else if (s.cur_rad > 0.0) {
      this_wide_aim = next_wide_aim;
      this_wide_start = next_wide_start;
      this_in_aim = next_in_aim;
      this_cd = next_cd;
      this_cs = next_cs;

      if (s.nex_rad == 0.0) {
	next_cs = MAX_SPD;
      }

      else if (s.nex_rad > 0.0) {
	double_corner = 1;
	this_cs *= CONTROL_TURN;
	theta = s.nex_len / 2;
	next_wide_aim = WIDTH - OUT_BUFF;
	next_in_aim = WIDTH - OUT_BUFF;
	next_r = s.nex_rad + s.to_lft;
	next_cs = calc_corn0(s.nex_rad,s.to_lft);
	next_cd = next_r * theta;
      }
   }
  }

//
// Putting all the pieces together:
//
  if (s.cur_rad == 0) {
    brake_lim = BRK_SLIP_LIM;
    bdst = brak_dist (s.v, next_cs, DEPTH);
    if (s.to_end > bdst + ci.dive) {
      lane = divide(s.to_lft + lane_glide(togo_start, s.to_end, next_wide_start, next_wide_aim),2);
      speed = s.v + 50.0;

      steer_gain = STRT_GAIN;
      damp_gain =  STRT_DAMP;
      bias = 0;
		}
    else if (s.to_end > ci.dive) {
      lane = s.to_lft;
      speed = next_cs;
      steer_gain = 0;
      damp_gain = 0;
      bias = 0;
    }
    else {
      lane = divide (s.to_lft + lane_glide(next_cd, next_cd - ci.dive + s.to_end, next_wide_aim, next_in_aim),2); 
      speed = next_cs;
      steer_gain = CORN_GAIN;
      damp_gain =  CORN_DAMP;
      bias = s.nex_rad>0 ? atan(divide(BIG_SLIP,speed)):-atan(divide(BIG_SLIP,speed));
    }


  }
  else if (s.cur_rad > 0.0) {
    brake_lim = TURN_BRK_SLIP_LIM;
    if (s.to_end < (s.cur_len/2)) {
      if ((s.to_end < EXIT) && (s.nex_rad == 0.0)) {
	lane = s.to_lft;
	lane += DRIFT;
	speed = s.v + 50.0;
	steer_gain = CORN_GAIN;
	damp_gain = CORN_DAMP;
	bias = atan(divide(BIG_SLIP,speed));
      }
      else {
	lane = lane_glide (s.cur_len/2, s.to_end, this_in_aim, this_wide_aim);
	speed = min(double_corner ? this_cs * CONTROL_EXIT : this_cs,next_cs);
	steer_gain = CORN_GAIN;
	damp_gain =  CORN_DAMP;
	bias = atan(divide(BIG_SLIP,speed));
      }
    }
    else {
      to_apex = s.to_end - (s.cur_len/2);
      lane = lane_glide (s.cur_len/2, to_apex, this_wide_start, this_in_aim);
      speed = this_cs;
      steer_gain = CORN_GAIN;
      damp_gain =  CORN_DAMP;
      bias = atan(divide(BIG_SLIP,speed));
    }
  }


  // At this point we have calculated values for all the variables we need to ask for
  // a velocity and a direction:

  vc = calc_vc(speed,s.v,brake_lim);

  alpha = steer_gain * (divide((s.to_lft - lane),WIDTH)) - damp_gain * divide(s.vn,s.v) + bias;


//
//              CHECK IF WE'VE BEEN MOVED FAR OFF OUR LINE OR ARE EXITING PITS
//


  if (s.to_lft < 0.0 || s.to_rgt < 0 || pit_exit){
		if (s.to_lft < 0.0) alpha-=.3;
		if (s.to_rgt < 0) alpha+=.3;
      if (s.out_pits)   pit_exit= 1;
		if (pit_exit){
			if (s.v<corner_spd(s.cur_rad))  vc = MAX_SPD;
		if (s.to_lft < PIT_BUFF)        alpha -=.05;
		else pit_exit=0;}

  }


  //  So we figure it out and pass it along, though it goes through avoid_cars first:

  result.vc = vc;
  result.alpha = alpha;
  avoid_cars(s, result);

// Pit Instructions

  result.request_pit=0;
 if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 5)){
		result.request_pit = 1;
		result.repair_amount = s.damage;
		result.fuel_amount = min(MAX_FUEL, s.laps_to_go*track.length*.0003);
	}
  return result;
}



